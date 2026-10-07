#pragma once
#include <Arduino.h>
#include <stdint.h>
class PiJoystick {
public:
  uint8_t pin;
  enum class State : short { down = -1, idle = 0, up = 1 };

  PiJoystick(uint8_t pin) : pin(pin) {} // 默认状态下无需在analogRead前设置模拟引脚的状态 https://docs.arduino.cc/learn/microcontrollers/analog-input/
  State read() {
    int r = analogRead(pin);
    return r <= 400 ? State::down
         : r <= 600 ? State::idle
         : State::up;
  }
};