#include "task5.h"
#include <Arduino.h>
#include "kinematics/kinematics.h"

namespace task5::curve {
  constexpr unsigned long deltaTime = 1000; // millisecond，每两点间的时间
  size_t pointsIndex = 0;

  float polynomials[3][4][4]; // 分别基于x y z，4个式子，每个式子4个参数

  void setup() {
    pointsIndex = 0;
    multiMoveSetup(0UL);
    if (!task5::record_finished) {
      return;
    }
    else {
      auto &p = task5::points;
      float dp[3][4]; // 分别基于x y z，4个值
      for (uint8_t xyz = 0; xyz < 3; ++xyz) { // 对于x y z
        for (uint8_t i = 0; i < 4; ++i) {
          dp[xyz][i] = p[i+1][xyz] - p[i][xyz];
        }
      }
      for (uint8_t xyz = 0; xyz < 3; ++xyz) {
        for (uint8_t i = 0; i < 4; ++i) {
          polynomials[xyz][i][0] = p[i][xyz];
        }
        float coefficient_1 = 3.0F/(56.0F*static_cast<float>(deltaTime));
        polynomials[xyz][0][1] = 0;
        polynomials[xyz][1][1] = coefficient_1*(15*dp[xyz][0] + 11*dp[xyz][1] + (-3)*dp[xyz][2] + 1*dp[xyz][3]);
        polynomials[xyz][2][1] = coefficient_1*((-4)*dp[xyz][0] + 12*dp[xyz][1] + 12*dp[xyz][2] + (-4)*dp[xyz][3]);
        polynomials[xyz][3][1] = coefficient_1*(1*dp[xyz][0] + (-3)*dp[xyz][1] + 11*dp[xyz][2] + 15*dp[xyz][3]);
        float coefficient_2 = coefficient_1/static_cast<float>(deltaTime);
        polynomials[xyz][0][2] = coefficient_2*(41*dp[xyz][0] + (-11)*dp[xyz][1] + 3*dp[xyz][2] + (-1)*dp[xyz][3]);
        polynomials[xyz][1][2] = coefficient_2*((-26)*dp[xyz][0] + 22*dp[xyz][1] + (-6)*dp[xyz][2] + 2*dp[xyz][3]);
        polynomials[xyz][2][2] = coefficient_2*(7*dp[xyz][0] + (-21)*dp[xyz][1] + 21*dp[xyz][2] + (-7)*dp[xyz][3]);
        polynomials[xyz][3][2] = coefficient_2*((-2)*dp[xyz][0] + 6*dp[xyz][1] + (-22)*dp[xyz][2] + 26*dp[xyz][3]);
        float coefficient_3 = coefficient_2/static_cast<float>(deltaTime)/3;
        polynomials[xyz][0][3] = coefficient_3*((-67)*dp[xyz][0] + 33*dp[xyz][1] + (-9)*dp[xyz][2] + 3*dp[xyz][3]);
        polynomials[xyz][1][3] = coefficient_3*(33*dp[xyz][0] + (-43)*dp[xyz][1] + 27*dp[xyz][2] + (-9)*dp[xyz][3]);
        polynomials[xyz][2][3] = coefficient_3*((-9)*dp[xyz][0] + 27*dp[xyz][1] + (-43)*dp[xyz][2] + 33*dp[xyz][3]);
        polynomials[xyz][3][3] = coefficient_3*(3*dp[xyz][0] + (-9)*dp[xyz][1] + 33*dp[xyz][2] + (-67)*dp[xyz][3]);
      }
    }
  }

  MoveState moveSingle(const float (&pStart)[3], const float (&pEnd)[3], unsigned long segTime, size_t index) {
    unsigned long nowTime = segTime;
    bool is_finished = false;
    if (segTime >= deltaTime) {
      nowTime = deltaTime;
      is_finished = true;
    }
    float x = polynomials[0][index][3]*pow(nowTime, 3) + polynomials[0][index][2]*sq(nowTime) + polynomials[0][index][1]*nowTime + polynomials[0][index][0],
          y = polynomials[1][index][3]*pow(nowTime, 3) + polynomials[1][index][2]*sq(nowTime) + polynomials[1][index][1]*nowTime + polynomials[1][index][0],
          z = polynomials[2][index][3]*pow(nowTime, 3) + polynomials[2][index][2]*sq(nowTime) + polynomials[2][index][1]*nowTime + polynomials[2][index][0];
    int a1, a2, a3;
    //bool reachable = ik_degree_fix(x, y, z, a1, a2, a3);
    bool reachable = ik_degree_fix(x, y, z, a1, a2, a3);
    if (!reachable) {
      return MoveState::unreachable;
    }
    bottom.write(a1);
    right.write(a2);
    left.write(a3);
    return is_finished ? MoveState::finished : MoveState::ongoing;
  }

  bool loop(unsigned long relativeTime) {
    if (!task5::record_finished) {
      Serial.print(F("no complete record exists"));
      return true;
    }
    return multiMoveLoop(relativeTime, task5::points, 5, pointsIndex, moveSingle);
  }

  void reset() {
  }
}