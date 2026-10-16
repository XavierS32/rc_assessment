#include <Arduino.h>
#include "buttons.h"

void ButtonState::setup(ButtonState::State state) {
  stableState = state;
  lastState = state;
  lastTime = millis();
}

bool ButtonState::stableChange(ButtonState::State state, unsigned long time, unsigned long debouncingTime) {
  // lastState = state;
  if ( state != lastState ) {
    lastTime = time;
    lastState = state;
  }

  if ( time - lastTime >= debouncingTime ) {
    // stableState = state;
    if ( state != stableState ) {
      stableState = state;
      return true;
    }
  }
  return false;
}

void PullUpButton::setup() {
  pinMode(pin, INPUT_PULLUP);
  buttonState.setup( (ButtonState::State)digitalRead(pin) );
}

bool PullUpButton::clicked() { // 如果以后扩展为有双击等，不能与其他同时使用，因为会直接修改state（即已经处理过，消费这次变化）
  if ( buttonState.stableChange( digitalRead(pin), millis() ) ) {
    if ( buttonState.stableState == ButtonState::released ) {
      return true;
    }
  }
  return false;
}