#include "task5.h"

namespace task5::direct {
  // 所有单位均为cm
  //constexpr float pStart[3] = {/*...14, 1, 12*//*0, 9, 12*//*0, 13, 10*//*0, 7, 12*//*0.71446, 7.27795, 12.14230*/0, 9, 12.14230-2.3-0.7};
  //constexpr float pEnd[3] = {/*...-10, 10, 12*//*0, 16, 12*//*0, 25, 10*//*1, 18, 13*/1.12734, 17.95594, 10.98319-2.3+0.5};
  constexpr float pStart[3] = {0, 9, 4.2};
  constexpr float pEnd[3] = {0, 20, 4.3};
  constexpr unsigned long time = 1000; // millisecond

  void setup() {
  }

  bool loop(unsigned long relativeTime) {
    MoveState moveState = moveDirect(pStart, pEnd, time, relativeTime);
    if (moveState == MoveState::unreachable) {
      Serial.println(F("unreachable(skip)"));
      return true;
    }
    return !(moveState == MoveState::ongoing);
  }

  void reset() {
  }
}