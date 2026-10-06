#include <Arduino.h>
#include <simple_timer.h>

namespace simple_timer {
  bool every(unsigned long &lastTime, unsigned long interval) {
    unsigned long now = millis();
    if ( now - lastTime >= interval ) {
      lastTime = now;
      return true;
    }
    else
      return false;
  }
}