#include <Arduino.h>
#include <Servo.h>
#include <simple_timer.h>
#include <pi_joystick.h>
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
        setServo( actions[actionIndex][subIndex], lastMoveTime );
        state = State::waitForReach;
        break; // [[gnu::fallthrough]]; // 此处一定会等待一段时间，故没必要做fallthrough性能优化
      case State::waitForReach:
        if ( simple_timer::every(lastMoveTime, 1800) ) {
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

  RecordSM::Rt_t RecordSM::run(ButtonState &clickedButton, bool reset, ActionRecord *(&actionRecords), size_t &actionRecordsSize, size_t &actionRecordsIndex) {
    if (reset) {
      reset_fsm();
      return Rt_t::on_cpl;
    }
    else if (clickedButton == ButtonState::none
          || clickedButton == ButtonState::second);
    else if (clickedButton == ButtonState::first
          || clickedButton == ButtonState::third) {
      ignoreButtonMsg(clickedButton, "Record");
    }
    else if (clickedButton == ButtonState::fourth) {
      reset_fsm();
      return Rt_t::restore;
    }

    switch (state) {
      case State::start:
        {
          // 亮信号灯，打印信息
          digitalWrite(3, HIGH);
          Serial.println("record start");
          // 初始化每次判定移动移动度数的计时器
          lastMoveTime = millis();
          // 将四个摇杆维度初始状态均设为置中
          for (auto &i : joystickStates) {
            i = PiJoystick::State::idle;
          }
          // 初始化舵机当前目标位置
          target_bottom_x1000 = static_cast<long>(bottom.read()) * 1000L;
          target_left_x1000 = static_cast<long>(left.read()) * 1000L;
          target_right_x1000 = static_cast<long>(right.read()) * 1000L;
          target_gripper_x1000 = static_cast<long>(gripper.read()) * 1000L;
          // 将index设为0，从头开始重新写入
          actionRecordsIndex = 0;
          // log第一个状态
          startTime = millis();
          bool succeed = addRecord( ActionRecord{target_bottom_x1000,
                                                  target_left_x1000,
                                                  target_right_x1000,
                                                  target_gripper_x1000,
                                                  0}, actionRecords, actionRecordsSize, actionRecordsIndex );
          if ( !succeed ) {
            Serial.println("OOM, record stops");
            reset_fsm();
            return Rt_t::on_cpl;
          }
          state = State::record;
        }
        [[gnu::fallthrough]];
      case State::record:
        {
          unsigned long nowTime = millis();
          unsigned long deltaTime = nowTime - lastMoveTime;

          // angle per second = angle per millisecond * 1000
          target_bottom_x1000 = constrain( target_bottom_x1000 + static_cast<short>(joystickStates[0])*static_cast<signed long>(deltaTime)*speed, 0L, 180000L );
          target_left_x1000 = constrain( target_left_x1000 + static_cast<short>(joystickStates[1])*static_cast<signed long>(deltaTime)*speed, 0L, 110000L );
          target_right_x1000 = constrain( target_right_x1000 + static_cast<short>(joystickStates[2])*static_cast<signed long>(deltaTime)*speed, 50000L, 110000L );
          target_gripper_x1000 = constrain( target_gripper_x1000 + static_cast<short>(joystickStates[3])*static_cast<signed long>(deltaTime)*speed, 0L, 77000L );
          // static unsigned long lt = millis();
          // if (simple_timer::every(lt, 800)){char buf[64];
          // sprintf(buf, "target_x1000: %ld\t%ld\t%ld\t%ld\n", target_bottom_x1000, target_left_x1000, target_right_x1000, target_gripper_x1000);
          // Serial.println(buf);
          // char buf2[64];
          // sprintf(buf2, "      target: %ld\t%ld\t%ld\t%ld\n", target_bottom_x1000/1000, target_left_x1000/1000, target_right_x1000/1000, target_gripper_x1000/1000);
          // Serial.println(buf2);}
          bottom.write( target_bottom_x1000 / 1000 );
          left.write( target_left_x1000 / 1000 );
          right.write( target_right_x1000 / 1000 );
          gripper.write( target_gripper_x1000 / 1000 );

          lastMoveTime = nowTime;

          // 处理button2
          if (clickedButton == ButtonState::second) {
            // 记录停止时的状态
            bool succeed = addRecord( ActionRecord{target_bottom_x1000,
                                                   target_left_x1000,
                                                   target_right_x1000,
                                                   target_gripper_x1000,
                                                   nowTime - startTime}, actionRecords, actionRecordsSize, actionRecordsIndex );
            if (!succeed) {
              Serial.println("OOM, record stops");
              reset_fsm();
              return Rt_t::on_cpl;
            }
            Serial.println("record stops");
            reset_fsm();
            return Rt_t::on_cpl;
          }
          // Serial.println("testing");
          // when leaving the state: digitalWrite(3, LOW);

          PiJoystick::State nowJoystickStates[4];
          nowJoystickStates[0] = bottomStick.stableRead();
          nowJoystickStates[1] = leftStick.stableRead();
          nowJoystickStates[2] = rightStick.stableRead();
          nowJoystickStates[3] = gripperStick.stableRead();
          // Serial.print((int)nowJoystickStates[0], DEC);
          // Serial.print(" ");
          // Serial.print((int)nowJoystickStates[1], DEC);
          // Serial.print(" ");
          // Serial.print((int)nowJoystickStates[2], DEC);
          // Serial.print(" ");
          // Serial.println((int)nowJoystickStates[3], DEC);
          // Serial.println();

          if ( !array_equal(nowJoystickStates, joystickStates) ) {
            // log the servo angles
            Serial.println("State changed");
            bool succeed = addRecord( ActionRecord{target_bottom_x1000,
                                                   target_left_x1000,
                                                   target_right_x1000,
                                                   target_gripper_x1000,
                                                   nowTime - startTime}, actionRecords, actionRecordsSize, actionRecordsIndex );
            if (!succeed) {
              Serial.println("OOM, record stops");
              reset_fsm();
              return Rt_t::on_cpl;
            }
            array_copy(nowJoystickStates, joystickStates); // 其实声明两个数组，然后用指针互换更高效
          }
        }
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
          setServo(90, 90, 90, 0, lastMoveTime);
          state = State::waitForReach;
          break; // [[gnu::fallthrough]]; // 此处一定会等待一段时间，故没必要做fallthrough性能优化
        case State::waitForReach:
          if ( simple_timer::every(lastMoveTime, 1800) ) {
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
          RecordSM::Rt_t subSM_rt = recordSM.run(clickedButton, reset, actionRecords, actionRecordsSize, actionRecordsIndex);
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
    pinMode(3, OUTPUT);
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

  // 自动尝试申请内存来储存记录，如果申请失败，返回false，成功返回true并更新储存状态
  // TODO：设计思考：程序中没有其他地方用heap（如果用的其他库中也没有的话），所以这里的realloc大概率不会出现碎片问题
  // 然而，这也意味着这里没有理由使用动态的realloc，静态数组已经可以满足要求，而且高度可控
  // avr中缺乏内存保护机制，因此stack可能会静默与heap互相覆盖，导致非常诡异的问题，所以这里无硬上限的realloc其实有风险
  // 因此实际上应该设计为经过大小计算给stack留下足够空间的*静态*数组。有空的时候会改写这里
  bool addRecord(ActionRecord actionRecord, ActionRecord *(&actionRecords), size_t &actionRecordsSize, size_t &actionRecordsIndex) {
    constexpr size_t newSizeOneTime = 10;
    if (actionRecordsIndex == actionRecordsSize) {
      ActionRecord *tmp;
      tmp = static_cast<ActionRecord*>( realloc( static_cast<void*>(actionRecords), sizeof(ActionRecord) * (actionRecordsSize + newSizeOneTime) ) );
      if ( tmp == nullptr ) {
        return false;
      }
      else {
        actionRecords = tmp;
        actionRecordsSize += newSizeOneTime;
      }
    }
    actionRecords[actionRecordsIndex] = actionRecord;
    actionRecordsIndex++;
    return true;
  }
}