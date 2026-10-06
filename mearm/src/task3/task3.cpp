#include <Arduino.h>
#include <Servo.h>
#include <simple_timer.h>
#include "task3.h"

namespace task3 {
  void setServo(int const (&angles)[4], unsigned long &lastTime) {
    bottom.write(angles[0]);
    left.write(angles[1]);
    right.write(angles[2]);
    gripper.write(angles[3]);
    // Serial.print("set bottom: ");
    // Serial.print(angles[0], DEC);
    // Serial.print(" left: ");
    // Serial.print(angles[1], DEC);
    // Serial.print(" right: ");
    // Serial.print(angles[2], DEC);
    // Serial.print(" gripper: ");
    // Serial.println(angles[3], DEC);
    lastTime = millis();
  }

  void ignoreButtonMsg(ButtonState const &clickedButton, char const*const name) {
    Serial.print("Busy running ");
    Serial.print(name);
    Serial.print(", button \'");
    Serial.print(static_cast<int>(clickedButton), DEC);
    Serial.println("\' ignored.");
  }

  LoopMovingSM::Rt_t LoopMovingSM::run(ButtonState &clickedButton) {
    if (clickedButton == ButtonState::none);
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
          reset_fsm();
          return Rt_t::on_cpl;
        }
        else {
          subIndex++;
          reset_fsm();
        }
        break;
    }
    return Rt_t::on_going;
  }

  Task3SM::Rt_t Task3SM::run(ButtonState &clickedButton) {
    switch ( state ) {
      case State::start:
        state = State::idle;
        [[gnu::fallthrough]];
      case State::idle:
        if (clickedButton == ButtonState::none);
        else if (clickedButton == ButtonState::first) {
          state = State::loopMoving;
        }
        else if (clickedButton == ButtonState::second) {
          state = State::recording;
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
          LoopMovingSM::Rt_t subSM_rt = loopMovingSM.run(clickedButton);
          if (subSM_rt == LoopMovingSM::Rt_t::restore) {
            state = State::restore;
            break;
          }
          else if (subSM_rt == LoopMovingSM::Rt_t::on_cpl) {
            reset_fsm();
            return Rt_t::on_going;
          }
          else if (subSM_rt == LoopMovingSM::Rt_t::on_going) {
            break;
          }
        }
        break;
      case State::recording:
        break;
      case State::play:
        break;
      case State::restore:
        break;
    }
    return Rt_t::on_going;
  }

  void setup() {
    Serial.println("task3 start");
  }

  bool loop() {
    ButtonState clickedButton = ButtonState::none;
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
          // TODO 在状态机内有序释放退出（如果录制部分直接退出会出现问题的话）
          return true;
          break;
        default:
          break;
      }
    }

    static Task3SM rootSM{};
    rootSM.run(clickedButton);

    return false;
  }
}