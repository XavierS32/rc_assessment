#include "task5.h"
#include <Arduino.h>

namespace task5::polyline {
  constexpr unsigned long deltaTime = 3000; // millisecond，每两点间的时间
  size_t pointsIndex = 0;

  void setup() {
    pointsIndex = 0;
    multiMoveSetup(0UL);
  }

  bool loop(unsigned long relativeTime) {
    if (!task5::record_finished) {
      Serial.print(F("no complete record exists"));
      return true;
    }
    return multiMoveLoop(relativeTime, task5::points, 5, pointsIndex,
      [](const float (&pStart)[3], const float (&pEnd)[3], unsigned long relativeTime, size_t index) {
        auto rt = moveDirect(pStart, pEnd, deltaTime, relativeTime);
        Serial.print(pStart[0]);
        Serial.print(' ');
        Serial.print(pStart[1]);
        Serial.print(' ');
        Serial.print(pStart[2]);
        Serial.println(' ');
        return rt;
      }, true);
  }

  void reset() {
  }
}