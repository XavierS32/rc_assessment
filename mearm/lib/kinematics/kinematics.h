#pragma once
#include <Arduino.h>

#define radian2degree(x) ((x) / PI * 180)
#define r2d(x) radian2degree(x)

void fk(float a1, float a2, float a3, float &x, float &y, float &z);

void ik(float x, float y, float z, float &a1, float &a2, float &a3);

inline void ik_degree(float x, float y, float z, float &a1, float &a2, float &a3) {
  ik(x, y, z, a1, a2, a3);
  a1 = r2d(a1);
  a2 = r2d(a2);
  a3 = r2d(a3);
}