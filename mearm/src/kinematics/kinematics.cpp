#include <Arduino.h>
#include "kinematics/kinematics.h"

// 单位均为cm
// x2 = p1_2org[0], y2 = p1_2org[1], z2 = p1_2org[2]; // l1 = |p1_2org|
float x2 = 1.5F, y2 = -0.5F, z2 = 3.0F;
float l2 = 8.0F, l3 = 8.0F, l4 = 5.0F;

void fk(float a1, float a2, float a3, float &x, float &y, float &z) {
  float t1 = a1, t2 = PI - a2, t3 = HALF_PI - a3;
  float u = l2*cosf(t2) + l3*cosf(t3) + l4, v = l2*sinf(t2) - l3*sinf(t3);
  float s1 = sinf(t1), c1 = cosf(t1);
  x = c1*(u + x2) - s1*y2;
  y = s1*(u + x2) + c1*y2;
  z = v + z2;
}

// 注意，没有电机角度限位检测
bool ik(float x, float y, float z, float &a1, float &a2, float &a3) {
  float k_sq = sq(x) + sq(y) - sq(y2);
  if (!(k_sq >= 0.0f)) return false; // gpt changes after code review
  float k = sqrt(k_sq);
  a1 = atan2f( k*y - y2*x, k*x + y2*y );
  float u = k - x2, v = z - z2;
  float d_sq = sq(u - l4) + sq(v), d = sqrt(d_sq);
  if (!(d > 1e-5f)) return false; // gpt changes after code review
  float phi = atan2f(v, u - l4);
  float A = ( sq(l2) + sq(l3) - d_sq ) / ( 2.0F * l2 * l3 ), B = ( sq(l3) - sq(l2) + d_sq ) / ( 2.0F * l3 * d );
  constexpr float EPS = 1e-5f; // 允许微小浮点误差 // gpt changes after code review
  if (!(A >= -1.0f-EPS && A <= 1.0f+EPS &&
        B >= -1.0f-EPS && B <= 1.0f+EPS))
    return false;
  A = constrain(A, -1.0f, 1.0f);
  B = constrain(B, -1.0f, 1.0f);
  float t3 = acosf(B) - phi, t2 = acosf(-A) - t3;
  a2 = PI - t2;
  a3 = HALF_PI - t3;
  return true;
}