#include <Arduino.h>
#include <kinematics.h>

// 单位均为cm
// x2 = p1_2org[0], y2 = p1_2org[1], z2 = p1_2org[2]; // l1 = |p1_2org|
float x2 = 1.3F, y2 = -0.5F, z2 = 3.0F;
float l2 = 8.0F, l3 = 8.0F, l4 = 5.0F;

void fk(float a1, float a2, float a3, float &x, float &y, float &z) {
  float t1 = a1, t2 = PI - a2, t3 = HALF_PI - a3;
  float u = l2*cosf(t2) + l3*cosf(t3) + l4, v = l2*sinf(t2) - l3*sinf(t3);
  float s1 = sinf(t1), c1 = cosf(t1);
  x = c1*(u + x2) - s1*y2;
  y = s1*(u + x2) + c1*y2;
  z = v + z2;
}

// TODO: 限制检测
void ik(float x, float y, float z, float &a1, float &a2, float &a3) {
  float k = sqrt(sq(x) + sq(y) - sq(y2));
  a1 = atan2f( k*y - y2*x, k*x + y2*y );
  float u = k - x2, v = z - z2;
  float d_sq = sq(u - l4) + sq(v), d = sqrt(d_sq);
  float phi = atan2f(v, u - l4);
  float A = ( sq(l2) + sq(l3) - d_sq ) / ( 2.0F * l2 * l3 ), B = ( sq(l3) - sq(l2) + d_sq ) / ( 2.0F * l3 * d );
  float t3 = acosf(B) - phi, t2 = acosf(-A) - t3;
  a2 = PI - t2;
  a3 = HALF_PI - t3;
}