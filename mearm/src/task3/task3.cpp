#include <Arduino.h>
#include <Servo.h>
#include <simple_timer.h>
#include "task3.h"

namespace task3 {
  void setServo(int b, int l, int r, int g, unsigned long &lastTime) {
    bottom.write(b);
    left.write(l);
    right.write(r);
    gripper.write(g);
    // Serial.print("set bottom: ");
    // Serial.print(b, DEC);
    // Serial.print(" left: ");
    // Serial.print(l, DEC);
    // Serial.print(" right: ");
    // Serial.print(r, DEC);
    // Serial.print(" gripper: ");
    // Serial.println(g, DEC);
    lastTime = millis();
  }
  void setServo(int const (&angles)[4], unsigned long &lastTime) {
    setServo(angles[0], angles[1], angles[2], angles[3], lastTime);
  }

  void ignoreButtonMsg(ButtonState const &clickedButton, char const*const name) {
    Serial.print("Busy running ");
    Serial.print(name);
    Serial.print(", button \'");
    Serial.print(static_cast<int>(clickedButton), DEC);
    Serial.println("\' ignored.");
  }

  LoopMovingSM::Rt_t LoopMovingSM::run(ButtonState &clickedButton, bool reset) {
    if (reset) {
      reset_fsm();
      return Rt_t::on_cpl;
    }
    else if (clickedButton == ButtonState::none);
    else if (clickedButton == ButtonState::first
          || clickedButton == ButtonState::second
          || clickedButton == ButtonState::third) {
      ignoreButtonMsg(clickedButton, "LoopMoving");
    }
    else if (clickedButton == ButtonState::fourth) {
      reset_fsm();
      return Rt_t::restore;
    }

    switch (state) {
      case State::start:
        state = State::moveOnce;
        [[gnu::fallthrough]];
      case State::moveOnce:
        setServo( actions[actionIndex][subIndex], lastTime );
        state = State::waitForReach;
        break; // [[gnu::fallthrough]]; // 此处一定会等待一段时间，故没必要做fallthrough性能优化
      case State::waitForReach:
        if ( simple_timer::every(lastTime, 1800) ) {
          state = State::isEnd;
          [[gnu::fallthrough]];
        }
        else {
          break;
        }
      case State::isEnd:
        if (subIndex == actionsSize[actionIndex] - 1) {
          subIndex = 0;
          actionIndex = (actionIndex + 1) % Action_size;
          reset_keeping_state();
          return Rt_t::on_cpl;
        }
        else {
          subIndex++;
          state = State::moveOnce;
        }
        break;
    }
    return Rt_t::on_going;
  }

  RecordSM::Rt_t RecordSM::run(ButtonState &clickedButton, bool reset) {
    if (reset) {
      reset_fsm();
      return Rt_t::on_cpl;
    }
    else if (clickedButton == ButtonState::none
          || clickedButton == ButtonState::second);
    else if (clickedButton == ButtonState::first
          || clickedButton == ButtonState::third) {
      ignoreButtonMsg(clickedButton, "LoopMoving");
    }
    else if (clickedButton == ButtonState::fourth) {
      reset_fsm();
      return Rt_t::restore;
    }

    switch (state) {
      case State::start:
        state = State::test;
        [[gun::fallthrough]];
      case State::test:
        Serial.println("testing");
        break;
    }
    return Rt_t::on_going;
  }

  RestoreSM::Rt_t RestoreSM::run(ButtonState &clickedButton, bool reset) {
      if (reset) {
        reset_fsm();
        return Rt_t::on_cpl;
      }
      else if (clickedButton == ButtonState::none);
      else {
        ignoreButtonMsg(clickedButton, "Restoring");
      }

      switch (state) {
        case State::start:
          state = State::restore;
          [[gnu::fallthrough]];
        case State::restore:
          setServo(90, 90, 90, 0, lastTime);
          state = State::waitForReach;
          break; // [[gnu::fallthrough]]; // 此处一定会等待一段时间，故没必要做fallthrough性能优化
        case State::waitForReach:
          if ( simple_timer::every(lastTime, 1800) ) {
            reset_fsm();
            return Rt_t::on_cpl;
          }
          break;
      }
      return Rt_t::on_going;
    }

  Task3SM::Rt_t Task3SM::run(ButtonState &clickedButton, bool reset) {
    switch ( state ) {
      case State::start:
        state = State::idle;
        [[gnu::fallthrough]];
      case State::idle:
        if (reset) {
          reset_fsm();
          return Rt_t::on_cpl;
        }
        else if (clickedButton == ButtonState::none);
        else if (clickedButton == ButtonState::first) {
          state = State::loopMoving;
        }
        else if (clickedButton == ButtonState::second) {
          state = State::record;
        }
        else if (clickedButton == ButtonState::third) {
          state = State::play;
        }
        else if (clickedButton == ButtonState::fourth) {
          state = State::restore;
        }
        break;
      case State::loopMoving:
        {
          // static LoopMovingSM subSM{};
          LoopMovingSM::Rt_t subSM_rt = loopMovingSM.run(clickedButton, reset);
          if (subSM_rt == LoopMovingSM::Rt_t::restore) {
            state = State::restore;
          }
          else if (subSM_rt == LoopMovingSM::Rt_t::on_cpl) {
            if (reset) {
              reset_fsm();
              return Rt_t::on_cpl;
            }
            state = State::idle;
          }
          else if (subSM_rt == LoopMovingSM::Rt_t::on_going);
        }
        break;
      case State::record:
        {
          RecordSM::Rt_t subSM_rt = recordSM.run(clickedButton, reset);
          if (subSM_rt == RecordSM::Rt_t::restore) {
            state = State::restore;
          }
          else if (subSM_rt == RecordSM::Rt_t::on_cpl) {
            if (reset) {
              reset_fsm();
              return Rt_t::on_cpl;
            }
            state = State::idle;
          }
          else if (subSM_rt == RecordSM::Rt_t::on_going);
        }
        break;
      case State::play:
        break;
      case State::restore:
        {
          RestoreSM::Rt_t subSM_rt = restoreSM.run(clickedButton, reset);
          if (subSM_rt == RestoreSM::Rt_t::on_cpl) {
            if (reset) {
              reset_fsm();
              return Rt_t::on_cpl;
            }
            state = State::idle;
          }
          else if (subSM_rt == RestoreSM::Rt_t::on_going);
        }
        break;
    }
    return Rt_t::on_going;
  }

  void setup() {
    Serial.println("task3 start");
  }

  bool loop() {
    // 本来更优雅的表达应该是 < ButtonState | bool > currentEvent，但是c++11无法方便的实现union type
    ButtonState clickedButton = ButtonState::none;
    bool reset = false;
    if (Serial.available() > 0) {
      char ch = Serial.read();
      // Serial.print("got serial command: ");
      // Serial.println((int)ch, DEC);
      switch (ch) {
        case 1:
          // Serial.println("got button pressed");
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
        default:
          break;
      }
    }

    // 若想要任何按钮信号/退出指令不被静默丢弃，任何一个状态中（或fallthrough中的至少一个）都需要确保处理按钮状态/退出指令
    static Task3SM rootSM{};
    Task3SM::Rt_t rootSM_rt = rootSM.run(clickedButton, reset);
    if (rootSM_rt == Task3SM::Rt_t::on_cpl) {
      return true;
    }
    else if (rootSM_rt == Task3SM::Rt_t::on_going);

    return false;
  }
}