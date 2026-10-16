#pragma once
#include <Arduino.h>
#include "arm_constrain.h"

#define radian2degreef(x) ((x) / PI * 180.0F)
#define r2df(x) radian2degreef(x)
#define degree2radianf(x) ((x) / 180.0F * PI)
#define d2rf(x) degree2radianf(x)

void fk(float a1, float a2, float a3, float &x, float &y, float &z);

bool ik(float x, float y, float z, float &a1, float &a2, float &a3);

inline bool ik_degree(float x, float y, float z, float &a1, float &a2, float &a3) {
  bool result = ik(x, y, z, a1, a2, a3);
  if (result) {
    a1 = r2df(a1);
    a2 = r2df(a2);
    a3 = r2df(a3);
    constexpr float EPS = 1e-5f; // 允许微小浮点误差
    if ((a2 >= RIGHT_MIN-EPS && a2 <= RIGHT_MAX+EPS)
     && (a3 >= LEFT_MIN-EPS  && a3 <= LEFT_MAX+EPS))
      return true;
    else  return false;
  }
  else  return false;
}

inline void fk_degree(float a1, float a2, float a3, float &x, float &y, float &z) {
  a1 = d2rf(a1);
  a2 = d2rf(a2);
  a3 = d2rf(a3);
  fk(a1, a2, a3, x, y, z);
}

// XYZ -> 标定后的舵机控制值
bool ik_degree_fix(
    float x, float y, float z,
    int &b, int &r, int &l
);

// 标定后的舵机控制值 -> 预测 XYZ
bool fk_degree_fix(
    int b, int r, int l,
    float &x, float &y, float &z
);

// inline bool ik_degree_fix(float x, float y, float z, float &a1, float &a2, float &a3) {
//   bool result = ik_degree(x, y, z, a1, a2, a3);
//   if (result) {
//     a1 = 200.0F / 187.0F * a1 - 1300.0F / 187.0F;
//     a2 = a2 - 10;
//     return true;
//   }
//   else  return false;
// }

// inline void fk_degree_fix(float a1, float a2, float a3, float &x, float &y, float &z) {
//   a1 = 187.0F / 200.0F * a1 + 6.5F;
//   a2 = a2 + 10;
//   fk_degree(a1, a2, a3, x, y, z);
// }