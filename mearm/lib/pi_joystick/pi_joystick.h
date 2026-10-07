#pragma once
#include <Arduino.h>
class PiJoystick {
public:
  uint8_t pin;
  enum class State : short { down = -1, idle = 0, up = 1 };
  bool reverse;
  State stableState = State::idle;

  // 默认状态下无需在analogRead前设置模拟引脚的状态 https://docs.arduino.cc/learn/microcontrollers/analog-input/
  PiJoystick(uint8_t pin, bool reverse = false) : pin(pin), reverse(reverse) {
    lastTime = millis();
  }
  State directRead() {
    int r = analogRead(pin);
    return r <= 300 ? (!reverse ? State::down : State::up)
         : r <= 700 ? State::idle
         : (!reverse ? State::up : State::down);
  }
  // 若需得到stableState，必须调用该函数进行更新
  bool stableChange(unsigned long debouncingTime = 10) {
    State nowState = directRead();
    unsigned long nowTime = millis();
    if ( nowState != lastState ) {
      lastTime = nowTime;
      lastState = nowState;
    }

    if ( nowTime - lastTime >= debouncingTime ) {
      if ( nowState != stableState ) {
        stableState = nowState;
        return true;
      }
    }
    return false;
  }
  State stableRead(unsigned long debouncingTime = 10) {
    stableChange(debouncingTime);
    return stableState;
  }
private:
  State lastState = State::idle;
  unsigned long lastTime;
};