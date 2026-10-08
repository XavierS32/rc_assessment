#include <Arduino.h>
#include <Servo.h>
#include <simple_timer.h>
#include <pi_joystick.h>
#include "task3.h"

namespace task3 {
  Record record;

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
    Serial.print("Busy running \'");
    Serial.print(name);
    Serial.print("\', button \'");
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

  RecordSM::Rt_t RecordSM::run(ButtonState &clickedButton, bool reset, Record &record, size_t actionRecordsSize) {
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
        // 亮信号灯，打印信息
        digitalWrite(3, HIGH);
        Serial.println("record start");

        // 将四个摇杆维度初始状态均设为置中
        for (auto &i : joystickStates) {
          i = PiJoystick::State::idle;
        }

        // 初始化record起始部分，及录制计数器
        record.is_complete_record = false;
        record.start_bottom = bottom.read();
        record.start_left = left.read();
        record.start_right = right.read();
        record.start_gripper = gripper.read();
        record.speed = 30;
        record.actionRecordsCount = 0;
        startTime = millis();

        // 使用record初始化移动计算，使其在录制和回放时保持一致
        move.set_init(record.start_bottom, record.start_left, record.start_right, record.start_gripper, 0, record.speed);
        state = State::record;
        [[gnu::fallthrough]];
      case State::record:
        {
          // 计算并移动到当前的理论位置
          // 仅用actionRecords中的数据来move（而move是“无副作用”（与外界隔离）的），故可以保证记录和播放时候理论轨迹完全一致
          long bottom_x1000, left_x1000, right_x1000, gripper_x1000; // computeThisTime得出的角度值是理论轨迹在当前的映射,computeThisTime本身无任何副作用
          unsigned long nowTime = millis();
          unsigned long relativeTime = nowTime - startTime;
          move.computeThisTime(relativeTime, bottom_x1000, left_x1000, right_x1000, gripper_x1000);
          bottom.write(bottom_x1000 / 1000L);
          left.write(left_x1000 / 1000L);
          right.write(right_x1000 / 1000L);
          gripper.write(gripper_x1000 / 1000L);

          // 处理再次按下button2结束录制
          if (clickedButton == ButtonState::second) {
            // 完善record终止部分
            record.is_complete_record = true;
            record.stopTime = relativeTime;
            reset_fsm();
            return Rt_t::on_cpl;
          }

          // 获取去抖后的摇杆状态
          PiJoystick::State nowJoystickStates[4];
          nowJoystickStates[0] = bottomStick.stableRead();
          nowJoystickStates[1] = leftStick.stableRead();
          nowJoystickStates[2] = rightStick.stableRead();
          nowJoystickStates[3] = gripperStick.stableRead();
          // 如果摇杆状态更新，更新录制数组，并以录制数组设定理想运动状态
          if ( !array_equal(nowJoystickStates, joystickStates) ) {
            // 更新录制数组
            bool full = !addRecord(ActionRecord{ nowJoystickStates[0], nowJoystickStates[1], nowJoystickStates[2], nowJoystickStates[3], relativeTime }, record, actionRecordsSize);
            if (full) { // 此处的行为是经过思考的，当数组满后，其实还可以录制一截沿当前状态运动的过程，直到下一次状态改变无法再被记录，因此此时停止更为合适
              Serial.println("memory full. record stop");
              // 完善record终止部分
              record.is_complete_record = true;
              record.stopTime = relativeTime;
              reset_fsm();
              return Rt_t::on_cpl;
            }
            else {
              Serial.print("memory usage: ");
              Serial.print(actionRecordsSize - record.actionRecordsCount, DEC);
              Serial.println(" actions left");
            }
            // 以录制数组设定理想运动状态
            ActionRecord &nowAR = record.actionRecords[record.actionRecordsCount - 1];
            move.changeState(static_cast<short>(nowAR.bottom),
                             static_cast<short>(nowAR.left),
                             static_cast<short>(nowAR.right),
                             static_cast<short>(nowAR.gripper), nowAR.time);
            // 更新当前摇杆状态以便下次判断
            array_copy(nowJoystickStates, joystickStates);
          }
        }
        break;
    }
    return Rt_t::on_going;
  }

  PlaySM::Rt_t PlaySM::run(ButtonState &clickedButton, bool reset, Record &record) {
    if (reset) {
      reset_fsm();
      return Rt_t::on_cpl;
    }
    else if (clickedButton == ButtonState::none);
    else if (clickedButton == ButtonState::first
          || clickedButton == ButtonState::second
          || clickedButton == ButtonState::third) {
      ignoreButtonMsg(clickedButton, "Play");
    }
    else if (clickedButton == ButtonState::fourth) {
      reset_fsm();
      return Rt_t::restore;
    }

    switch (state)
    {
      case State::start:
        // 首先，检测有无录制。注意，也有可能有录制count为0（什么都没动），这种情况下面也能正常处理
        if (!record.is_complete_record) {
          Serial.println("no complete record exists");
          reset_fsm();
          return Rt_t::on_cpl;
        }
        // 重设播放索引
        actionRecordsIndex = 0;
        // 初始化到开始录制的角度
        Serial.println("initlizing...");
        bottom.write(record.start_bottom);
        left.write(record.start_left);
        right.write(record.start_right);
        gripper.write(record.start_gripper);
        initStartTime = millis();
        state = State::waitForInitMove;
        [[gnu::fallthrough]];
      case State::waitForInitMove:
        if ( millis() - initStartTime < 1800 ) {
          break;
        }
        else {
          move.set_init(record.start_bottom, record.start_left, record.start_right, record.start_gripper, 0, record.speed);
          playStartTime = millis();
          Serial.println("play start");
          state = State::playAction;
        }
        [[gnu::fallthrough]];
      case State::playAction:
        {
          unsigned long nowTime = millis();
          unsigned long relativeTime = nowTime - playStartTime;

          // 如果到摇杆状态更新的时间了，和录制时相同的，以同样的录制数组设置相应理想运动状态
          // 注意，这里可能跳过了多个间隔，所以要用循环一次性设置到当前的最终状态
          // 而且，由于不确定是否跳过了间隔，所以必须先设置到合适的状态，再计算理论位置。
          // 操作顺序和录制时有出入，但是算法理论保证该时间所设定的理论转角与录制时的该时刻完全相同
          while (actionRecordsIndex < record.actionRecordsCount
                 && record.actionRecords[actionRecordsIndex].time <= relativeTime) {
            ActionRecord const &nowAR = record.actionRecords[actionRecordsIndex];
            move.changeState(static_cast<short>(nowAR.bottom),
                              static_cast<short>(nowAR.left),
                              static_cast<short>(nowAR.right),
                              static_cast<short>(nowAR.gripper), nowAR.time);
            ++actionRecordsIndex;
          }
          // 如果时间超过的结束时间，将目标位置设为结束的那一刻
          if (nowTime - playStartTime > record.stopTime) {
            relativeTime = record.stopTime;
          }

          // 计算并移动到当前的理论位置
          long bottom_x1000, left_x1000, right_x1000, gripper_x1000;
          move.computeThisTime(relativeTime, bottom_x1000, left_x1000, right_x1000, gripper_x1000);
          bottom.write(bottom_x1000 / 1000L);
          left.write(left_x1000 / 1000L);
          right.write(right_x1000 / 1000L);
          gripper.write(gripper_x1000 / 1000L);

          // 如果达到或超过结束时间，结束播放
          if (relativeTime == record.stopTime) {
            reset_fsm();
            return Rt_t::on_cpl;
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

  Task3SM::Rt_t Task3SM::run(ButtonState &clickedButton, bool reset, Record &record, size_t actionRecordsSize) {
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
          RecordSM::Rt_t subSM_rt = recordSM.run(clickedButton, reset, record, actionRecordsSize);
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
        {
          PlaySM::Rt_t subSM_rt = playSM.run(clickedButton, reset, record);
          if (subSM_rt == PlaySM::Rt_t::restore) {
            state = State::restore;
          }
          else if (subSM_rt == PlaySM::Rt_t::on_cpl) {
            if (reset) {
              reset_fsm();
              return Rt_t::on_cpl;
            }
            state = State::idle;
          }
          else if (subSM_rt == PlaySM::Rt_t::on_going);
        }
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
    Task3SM::Rt_t rootSM_rt = rootSM.run(clickedButton, reset, record, actionRecordsSize);
    if (rootSM_rt == Task3SM::Rt_t::on_cpl) {
      return true;
    }
    else if (rootSM_rt == Task3SM::Rt_t::on_going);

    return false;
  }

  bool addRecord(ActionRecord const &actionRecord, Record &record, size_t const actionRecordsSize) {
    if (record.actionRecordsCount >= actionRecordsSize) {
      return false;
    }
    else {
      record.actionRecords[record.actionRecordsCount++] = actionRecord;
      return true;
    }
  }
}