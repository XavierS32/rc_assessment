#include <Arduino.h>
#include <Servo.h>
#include <string.h> // avr-gcc缺少cstring
#include <prefix_matcher.h>
#include <pi_joystick.h>
#include "kinematics/kinematics.h"
#include "button_state/button_state.h"
#include "fsm.h"
#include "gripper/gripper.h"
#include "task3/task3.h"
#include "task5.h"

namespace task5 {
  SetupFunc setupFunc;
  LoopFunc loopFunc;
  ResetFunc resetFunc;

  // direct shape等为子程序，而非子状态机，按钮状态的处理在该状态机中完成
  ControlSM::Rt_t ControlSM::run(ButtonState const &clickedButton, bool reset, SetupFunc setupFunc, LoopFunc loopFunc, ResetFunc resetFunc) {
    switch (state)
    {
      case State::start:
        if (reset) {
          reset_fsm();
          return Rt_t::on_cpl;
        }
        state = State::move;
        setupFunc();
        baseTimeLength = 0;
        baseTime = millis();
        // update: relativeTime = baseTimeLength + millis() - baseTime (取整问题？……)
        [[gnu::fallthrough]];
      case State::move:
        {
          unsigned long nowTime = millis();
          unsigned long relativeTime = baseTimeLength + nowTime - baseTime;
          if (reset) {
            resetFunc();
            reset_fsm();
            return Rt_t::on_cpl;
          }
          else if (clickedButton == ButtonState::first) { // pause
            baseTimeLength = relativeTime; // 所有进入pause状态均需运行
            state = State::pause;
            break;
          }
          else if (clickedButton == ButtonState::third) { // cancel
            resetFunc(); // 非自然终止进入cancel均需运行
            state = State::stop;
            break;
          }
          else if (clickedButton == ButtonState::second
                || clickedButton == ButtonState::fourth) {
            task3::ignoreButtonMsg(clickedButton, "move");
          }
          bool result = loopFunc(relativeTime); // 返回true结束后应该如何处理？……
          if (result) {
            readyForStop = nowTime; // 所有入waitForStop均需运行
            state = State::waitBeforeStop;
          }
        }
        break;
      case State::pause:
        {
          unsigned long nowTime = millis();
          if (reset) {
            resetFunc();
            reset_fsm();
            return Rt_t::on_cpl;
          }
          else if (clickedButton == ButtonState::second) { // continue
            baseTime = nowTime; // 所有出pause状态均需运行
            state = State::move;
            break;
          }
          else if (clickedButton == ButtonState::third) { // cancel
            resetFunc(); // 非自然终止进入cancel均需运行
            baseTime = nowTime; // 所有出pause状态均需运行
            state = State::stop;
            break;
          }
          else if (clickedButton == ButtonState::first
                || clickedButton == ButtonState::fourth) {
            task3::ignoreButtonMsg(clickedButton, "pause");
          }
          // do nothing here
        }
        break;
      case State::waitBeforeStop:
        {
          if (millis() - readyForStop > 500) {
            state = State::stop;
            [[gnu::fallthrough]];
          }
          else  break;
        }
      case State::stop:
        {
          if (clickedButton == ButtonState::first
           || clickedButton == ButtonState::second
           || clickedButton == ButtonState::third
           || clickedButton == ButtonState::fourth) {
            task3::ignoreButtonMsg(clickedButton, "stop");
          }
          ButtonState tmp = ButtonState::none;
          task3::RestoreSM::Rt_t subSM_rt = restoreSM.run(tmp, reset, true);
          if (subSM_rt == task3::RestoreSM::Rt_t::on_cpl) {
            reset_fsm();
            return Rt_t::on_cpl;
          }
          else if (subSM_rt == task3::RestoreSM::Rt_t::on_going);
        }
        break;
      default:
        break;
    }
    return Rt_t::on_going;
  }

  // 在time*毫秒*内从pStart直线移动到pEnd。注意，传入的time不得为0
  MoveState moveDirect(float const (&pStart)[3], float const (&pEnd)[3], unsigned long const time, unsigned long const relativeTime) {
    unsigned long nowTime = relativeTime;
    bool is_finished = false;
    if (relativeTime >= time) { // 注意正确处理回绕 TODO
      nowTime = time;
      is_finished = true;
    }
    float x = map_UL_to_F(nowTime, 0, time, pStart[0], pEnd[0]),
          y = map_UL_to_F(nowTime, 0, time, pStart[1], pEnd[1]),
          z = map_UL_to_F(nowTime, 0, time, pStart[2], pEnd[2]);
    int a1, a2, a3;
    bool reachable = ik_degree_fix(x, y, z, a1, a2, a3);
    if (!reachable) {
      return MoveState::unreachable;
    }
    // 可及性检查 // 加上限位
    // TODO: 注意！舵机限位的公共处理（前面程序，使用自己逻辑处理的，暂时保留）
    bottom.write(a1);
    right.write(a2);
    left.write(a3);
    return is_finished ? MoveState::finished : MoveState::ongoing;
  }

  PrefixRule<SubProgramFuncs> rules[] = {
    {"d", SubProgramFuncs {
      [](char const* str) {
        setupFunc = direct::setup;
        loopFunc = direct::loop;
        resetFunc = direct::reset; },
      controlSMLoop}},
    {"s", SubProgramFuncs {
      [](char const* str) {
        setupFunc = shape::setup;
        loopFunc = shape::loop;
        resetFunc = shape::reset; },
      controlSMLoop}},
    {"r", SubProgramFuncs {
      [](char const* str) {
        record::setup(); },
      record::loop}},
    {"p", SubProgramFuncs {
      [](char const* str) {
        setupFunc = polyline::setup;
        loopFunc = polyline::loop;
        resetFunc = polyline::reset; },
      controlSMLoop}},
    {"c", SubProgramFuncs {
      [](char const* str) {
        setupFunc = curve::setup;
        loopFunc = curve::loop;
        resetFunc = curve::reset; },
      controlSMLoop}},
    {"go", SubProgramFuncs {
      [](char const* str) {
        gripper.set(false); },
      []() { return true; }}},
    {"gc", SubProgramFuncs {
      [](char const* str) {
        gripper.set(true); },
      []() { return true; }}}
  };

  SubProgramFuncs fallback {
    [](char const* str) {
      Serial.println(("invalid subProgram")); },
    []() { return true; }
  };

  auto prefixMatcher = makePrefixMatcher(rules, fallback);

  SubProgramFuncs subProgramFuncs{};

  void setup(char const*const str) {
    Serial.println(F("task5 setup"));
    // float x = 10, y = 8, z = 7;
    // float a1, a2, a3;
    // ik_degree(x, y, z, a1, a2, a3);
    // Serial.print(a1, 5);
    // Serial.print(" ");
    // Serial.print(a2, 5);
    // Serial.print(" ");
    // Serial.print(a3, 5);
    // Serial.print(" ");
    // bottom.write(a1);
    // right.write(a2);
    // left.write(a3);
    // float a, b, c;
    // fk_degree(a1, a2, a3, a, b, c);
    // Serial.print(a, 5);
    // Serial.print(" ");
    // Serial.print(b, 5);
    // Serial.print(" ");
    // Serial.print(c, 5);
    // Serial.print(" ");
    // direct::setup();
  
    // 每个过程都要实现按钮控制功能，故统一放在控制状态机中
    // 可达范围全局更新
    // 不可达异常处理

    char const *subCommand = strchr(str, ' ');
    if (subCommand != nullptr) {
      while (*subCommand != '\0') {
        if (*subCommand != ' ') {
          subProgramFuncs = prefixMatcher.match(subCommand);
          subProgramFuncs.setup(str);
          return;
        }
        subCommand++;
      }
    }
    subProgramFuncs = fallback;
    subProgramFuncs.setup(str);
  }

  bool loop() {
    return subProgramFuncs.loop();
  }

  PiJoystick bottomStick{A0, true};
  PiJoystick leftStick{A1};
  PiJoystick rightStick{A3, true};
  PiJoystick gripperStick{A2};

  bool controlSMLoop() {
    // 本来更优雅的表达应该是 < ButtonState | bool > currentEvent，但是c++11无法方便的实现union type
    ButtonState clickedButton = ButtonState::none;
    bool reset = false;
    if (Serial.available() > 0) {
      char ch = Serial.read();
      switch (ch) {
        case 1:
          clickedButton = ButtonState::first;
          break;
        case 2:
          clickedButton = ButtonState::second;
          break;
        case 3:
          clickedButton = ButtonState::third;
          break;
        case 4:
          clickedButton = ButtonState::fourth;
          break;
        case 'q':
          reset = true;
          break;
      }
    }
    
    // 若想要任何按钮信号/退出指令不被静默丢弃，任何一个状态中（或fallthrough中的至少一个）都需要确保处理按钮状态/退出指令
    static ControlSM rootSM{};
    ControlSM::Rt_t rootSM_rt = rootSM.run(clickedButton, reset, setupFunc, loopFunc, resetFunc);
    if (rootSM_rt == ControlSM::Rt_t::on_cpl) {
      return true;
    }
    else if (rootSM_rt == ControlSM::Rt_t::on_going);

    return false;
  }

  unsigned long lastRelativeTime; // multiMove[Setup|Loop]不可重入
  bool multiMoveLoop_pausing = false;
  unsigned long multiMoveLoop_pauseStartTime; // multiMoveLoop不可重入
  void multiMoveSetup(unsigned long relativeTime) {
    lastRelativeTime = relativeTime;
    multiMoveLoop_pausing = false;
  }

  // 加入pause后，其实用状态机实现远更加清晰
  bool multiMoveLoop(unsigned long relativeTime, float points[][3], size_t pointsSize, size_t &pointsIndex, MoveFunc moveFunc, bool pause, unsigned long pauseTime) {
    if (multiMoveLoop_pausing) {
      if (relativeTime - multiMoveLoop_pauseStartTime >= pauseTime) {
        multiMoveLoop_pausing = false;
        lastRelativeTime = relativeTime;
      }
      else {
        return false;
      }
    }

    if (pointsIndex+1 >= pointsSize ) {
      lastRelativeTime = relativeTime;
      return true;
    }

    MoveState moveState = moveFunc(points[pointsIndex], points[pointsIndex + 1], relativeTime - lastRelativeTime, pointsIndex);
    if (moveState == MoveState::unreachable) {
      Serial.print(F("unreachable(stop)"));
      return true;
    }
    else if (moveState == MoveState::finished) {
      pointsIndex++;
      lastRelativeTime = relativeTime;
      if (pointsIndex +1 >= pointsSize) {
        return true;
      }
      if (pause)  multiMoveLoop_pausing = true;
      multiMoveLoop_pauseStartTime = relativeTime;
      return false;
    }
    else if (moveState == MoveState::ongoing);

    return false;
  }

}
// 遥杆始终可以控制爪子的开闭done
// 调，真的把物体拿起来，是要花时间的！……

    // PiJoystick::State state = gripperStick.stableRead();
    // if (state == PiJoystick::State::down) gripper.set(false);
    // else if (state == PiJoystick::State::up)  gripper.set(true);

// 移动到初始位置