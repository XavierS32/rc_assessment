#include <Arduino.h>
#include "kinematics/kinematics.h"
#include <math.h>

// 单位均为cm
// x2 = p1_2org[0], y2 = p1_2org[1], z2 = p1_2org[2]; // l1 = |p1_2org|
float x2 = 1.5F, y2 = /*-0.5F*/ 0, z2 = 3.0F;
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

// 以下内容为在gpt辅助完成标定后，由gpt实现（如果时间充足我就自己琢磨了）
// 已标定的舵机映射参数
constexpr float BASE_GAIN   = 16.0f / 15.0f;
constexpr float RIGHT_GAIN  = 16.0f / 15.0f;
constexpr float LEFT_GAIN   = 6.0f / 5.0f;

constexpr float BASE_OFFSET  = 1.0f;
constexpr float RIGHT_OFFSET = -16.0f;
constexpr float LEFT_OFFSET  = -15.0f;

// 根据官方默认关节范围换算的控制值范围
// 后续可根据实际机械限位调整
// self: 与arm_constrain.h不同的是，这里是针对正逆解的限制，arm_constrain.h针对的是实际移动，范围更广
// constexpr int BASE_SERVO_MIN  = 49;
// constexpr int BASE_SERVO_MAX  = 145;
// constexpr int RIGHT_SERVO_MIN = 32;
// constexpr int RIGHT_SERVO_MAX = 128;
// constexpr int LEFT_SERVO_MIN  = 39;
// constexpr int LEFT_SERVO_MAX  = 147;

#include <arm_constrain.h>

// XYZ -> 标定后的舵机控制值
bool ik_degree_fix(
    float x, float y, float z,
    int &b, int &r, int &l
) {
    float a1, a2, a3;

    if (!ik_degree(x, y, z, a1, a2, a3))
      return false;

    if (!isfinite(a1) || !isfinite(a2) || !isfinite(a3))
      return false;

    int tb = (int)roundf(BASE_GAIN  * a1 + BASE_OFFSET);
    int tr = (int)roundf(RIGHT_GAIN * a2 + RIGHT_OFFSET);
    int tl = (int)roundf(LEFT_GAIN  * a3 + LEFT_OFFSET);

    // if (tb < BASE_SERVO_MIN || tb > BASE_SERVO_MAX ||
    //     tr < RIGHT_SERVO_MIN || tr > RIGHT_SERVO_MAX ||
    //     tl < LEFT_SERVO_MIN || tl > LEFT_SERVO_MAX)
    //     return false;
    if (tr < RIGHT_MIN || tr > RIGHT_MAX ||
        tl < LEFT_MIN || tl > LEFT_MAX)
      return false;

    b = tb;
    r = tr;
    l = tl;

    return true;
}

// 标定后的舵机控制值 -> 预测 XYZ
bool fk_degree_fix(
    int b, int r, int l,
    float &x, float &y, float &z
) {
    // if (b < BASE_SERVO_MIN || b > BASE_SERVO_MAX ||
    //     r < RIGHT_SERVO_MIN || r > RIGHT_SERVO_MAX ||
    //     l < LEFT_SERVO_MIN || l > LEFT_SERVO_MAX)
    //     return false;
    if (r < RIGHT_MIN || r > RIGHT_MAX ||
        l < LEFT_MIN || l > LEFT_MAX)
      return false;

    float a1 = (b - BASE_OFFSET) / BASE_GAIN;
    float a2 = (r - RIGHT_OFFSET) / RIGHT_GAIN;
    float a3 = (l - LEFT_OFFSET) / LEFT_GAIN;

    fk_degree(a1, a2, a3, x, y, z);

    return true;
}