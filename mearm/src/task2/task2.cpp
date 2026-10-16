#include <Arduino.h>
#include "task2.h"

namespace task2 {
  // Actions
  uint8_t const A[A_size][4] PROGMEM = {
    {90, 70, 112, 0},
    {90, 70, 148, 0},
    {90, 70, 148, 1},
    {90, 70, 53, 1},
    {90, 70, 53, 0}
  };

  uint8_t const B[B_size][4] PROGMEM = {
    {52, 65, 129, 0},
    //{52, 34, 129, 0},
    //{52, 34, 129, 1},
    {52, 35, 136, 0},
    {52, 35, 136, 1},
    {100, 50, 100, 1},
    {100, 34, 129, 1},
    {100, 34, 129, 0}
  };

  uint8_t const C[C_size][4] PROGMEM = {
    {113, 61, 137, 0},
    {113, 22, 137, 0},
    {113, 22, 137, 1},
    {100, 85, 96, 1},
    {29, 15, 133, 1},
    {29, 15, 133, 0}
  };

  void setup() {

  }

  bool loop() {
    // 本来更优雅的表达应该是 < Command | bool > currentEvent，但是c++11无法方便的实现union type
    Command command = Command::none;
    bool reset = false;
    if (Serial.available() > 0) {
      char ch = Serial.read();
      switch (ch) {
        case 'q':
          reset = true;
          break;
        case 'A':
        case 'a':
          command = Command::A;
          break;
        case 'B':
        case 'b':
          command = Command::B;
          break;
        case 'C':
        case 'c':
          command = Command::C;
          break;
      }
    }

    // 若想要任何指令不被静默丢弃，任何一个状态中（或fallthrough中的至少一个）都需要确保处理指令
    static Task2SM rootSM{};
    Task2SM::Rt_t rootSM_rt = rootSM.run(command, reset);
    if (rootSM_rt == Task2SM::Rt_t::on_cpl) {
      return true;
    }
    else if (rootSM_rt == Task2SM::Rt_t::on_going);

    return false;
  }

  Task2SM::Rt_t Task2SM::run(Command command, bool reset) {
    switch (state) {
      case State::start:
        state = State::idle;
        [[gnu::fallthrough]];
      case State::idle:
        if (reset) {
          reset_fsm();
          return Rt_t::on_cpl;
        }
        else if (command == Command::none);
        else if (command == Command::A) {
          action = A;
          actionSize = A_size;
          state = State::moving;
        }
        else if (command == Command::B) {
          action = B;
          actionSize = B_size;
          state = State::moving;
        }
        else if (command == Command::C) {
          action = C;
          actionSize = C_size;
          state = State::moving;
        }
        break;
      case State::moving:
        {
          if (command == Command::none);
          else if (command == Command::A
                || command == Command::B
                || command == Command::C) {
            Serial.println("Busy running, command ignored.");
          }
          MoveSeriesSM::Rt_t subSM_rt = moveSeriesSM.run(action, actionSize, reset);
          if (subSM_rt == MoveSeriesSM::Rt_t::on_cpl) {
            if (reset) {
              reset_fsm();
              return Rt_t::on_cpl;
            }
            state = State::idle;
          }
          else if (subSM_rt == MoveSeriesSM::Rt_t::on_going);
        }
        break;
    }
    return Rt_t::on_going;
  }

  MoveSeriesSM::Rt_t MoveSeriesSM::run(uint8_t const action[][4], size_t const actionSize, bool reset) {
    if (reset) {
      reset_fsm();
      return Rt_t::on_cpl;
    }

    switch (state) {
      case State::start:
        state = State::initTarget;
        [[gnu::fallthrough]];
      case State::initTarget:
        {
          uint8_t const (&subAction)[4] = action[subIndex];
          moveTo.initTarget(bottom.read(), left.read(), right.read(), gripper.readSetInt(),
                            pgm_read_byte(&subAction[0]), pgm_read_byte(&subAction[1]), pgm_read_byte(&subAction[2]), pgm_read_byte(&subAction[3]),
                            millis(), speed);
          state = State::moveTillEnd;
        }
        [[gnu::fallthrough]];
      case State::moveTillEnd:
        {
          unsigned long nowTime = millis();
          long bottom_x1000, left_x1000, right_x1000, gripper_x1000;
          if ( moveTo.move(nowTime, bottom_x1000, left_x1000, right_x1000, gripper_x1000) ) {
            state = State::isEnd;
            break;
          }
        }
        break;
      case State::isEnd:
        if (subIndex == actionSize - 1) {
          reset_fsm();
          return Rt_t::on_cpl;
        }
        else {
          subIndex++;
          state = State::initTarget;
        }
        break;
    }
    return Rt_t::on_going;
  }
}