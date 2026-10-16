#include "task5.h"
#include <Arduino.h>
#include "kinematics/kinematics.h"
  
  namespace task5::shape {
    // float points[4][3] = {{0, 9, 12.14230-2.3-0.7},
    //                       {1.12734, 17.95594, 10.98319-2.3+0.5},
    //                       {6.24834, 10.24097, 8.98714-2.3+1},
    //                       {-2, 9.5, 12.14230-2.3}};
    // float points[4][3] = {{0, 9, 4.2},
    //                       {1.127, 17.956, 4.2},
    //                       {6.248, 10.241, 4.2},
    //                       {0, 9, 4.2}};
    // float points[4][3] = {{-3, 19, 4.2},
    //                       {3, 19, 4.2},
    //                       {0, 2, 4.2},
    //                       {-3, 19, 4.2}};
    float points[5][3] = {{0,0,0},
                          {0, 11, 4.2},
                          {11, 13, 4.2},
                          {1, 16, 4.2},
                          {0, 11, 4.2}};
    size_t pointsIndex = 0;
    int deltaTime = 1000;
  
    void setup() {
      pointsIndex = 0;
      multiMoveSetup(0UL);
      int a1 = bottom.read();
      int a2 = right.read();
      int a3 = left.read();
      fk_degree_fix(a1, a2, a3, points[0][0], points[0][1], points[0][2]);
    }

    bool loop(unsigned long relativeTime) {
      return multiMoveLoop(relativeTime, points, 5, pointsIndex,
      [](const float (&pStart)[3], const float (&pEnd)[3], unsigned long relativeTime, size_t index) {
        return moveDirect(pStart, pEnd, deltaTime, relativeTime);
      }, true);
    }

    void reset() {
    }
  }