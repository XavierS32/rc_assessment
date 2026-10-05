#pragma once
#include <Arduino.h>

// 用于按钮防抖
struct ButtonState { 
  enum State { released = HIGH, pressed = LOW };
  State stableState, lastState;
  unsigned long lastTime;

  void setup(ButtonState::State state);
  bool stableChange(ButtonState::State state, unsigned long time, unsigned long debouncingTime = 10 /* ms */);
  bool stableChange(int state, unsigned long time) {
    return stableChange((ButtonState::State)state, time);
  }
};

// Usage:
// PullUpButton b1(PB10);
//
// void setup() {
//   b1.setup();
// }
//
// void loop() {
//   if ( b1.clicked() ) {
//       // ...
//   }
// }
class PullUpButton {
public:
  ButtonState buttonState;
  pin_size_t pin;

  PullUpButton(pin_size_t pin) : pin(pin) {}
  void setup();
  bool clicked();
};