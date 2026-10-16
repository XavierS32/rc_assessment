#pragma once
#include <Arduino.h>

namespace simple_timer {
  bool every(unsigned long &lastTime, unsigned long interval);
}