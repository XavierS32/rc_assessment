#include <Arduino.h>
#include "task2.h"

namespace task2 {
  // Actions
  int const A[A_size][4] = {
    {90, 12, 126, 0},
    {90, 12, 126, 78},
    {54, 42, 126, 78},
    {54, 42, 126, 0}
  };

  int const B[B_size][4] = {
    {90, 90, 90, 0}
  };

  int const C[C_size][4] = {
    {180, 90, 90, 0},
    {90, 90, 90, 0}
  };

  void setup() {

  }

  bool loop() {
    return true;
  }
}