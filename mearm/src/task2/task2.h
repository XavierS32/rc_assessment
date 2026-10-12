#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "gripper/gripper.h"
#include "fsm.h"

extern Servo bottom;
extern Servo left;
extern Servo right;
extern Gripper gripper;

namespace task2 {
  enum class Command : short { none = 0, A, B, C };

  // Actions
  constexpr size_t A_size = 4;
  extern int const A[A_size][4];

  constexpr size_t B_size = 1;
  extern int const B[B_size][4];

  constexpr size_t C_size = 2;
  extern int const C[C_size][4];

  class MoveTo {
  public:
    int target_bottom, target_left, target_right, target_gripper;
    long lastState_bottom_x1000, lastState_left_x1000, lastState_right_x1000, lastState_gripper_x1000; // 上一次状态转变那一刻电机角度*1000
    short bottom_direction, left_direction, right_direction, gripper_direction; // -1 0 1，运动方向
    unsigned long lastStateChangeTime; // 上一次状态转变那一刻的时间
    int speed; // angle per second
    // 设定基础状态
    void initTarget(int bottom, int left, int right, int gripper,
                  int target_bottom, int target_left, int target_right, int target_gripper,
                  unsigned long time, int speed) {
      lastState_bottom_x1000 = bottom * 1000L;
      lastState_left_x1000 = left * 1000L;
      lastState_right_x1000 = right * 1000L;
      lastState_gripper_x1000 = gripper * 1000L;
      lastStateChangeTime = time;
      this->speed = speed;
#define GET_DIRECTION(angle, target_angle) ((angle) < (target_angle) ? 1 : \
                                            (angle) > (target_angle) ? -1 : 0)
      bottom_direction = GET_DIRECTION(bottom, target_bottom);
      left_direction = GET_DIRECTION(left, target_left);
      right_direction = GET_DIRECTION(right, target_right);
      gripper_direction = GET_DIRECTION(gripper, target_gripper);
#undef GET_DIRECTION
      this->target_bottom = target_bottom;
      this->target_left = target_left;
      this->target_right = target_right;
      this->target_gripper = target_gripper;
    }
    // 计算给定时间的理想位置
    bool computeThisTime(unsigned long time, long &bottom_x1000, long &left_x1000, long &right_x1000, long &gripper_x1000) {
      unsigned long deltaTime = time - lastStateChangeTime;
      bool bottom_is_ready = computeSingleThisTime(deltaTime, bottom_x1000, lastState_bottom_x1000, bottom_direction, target_bottom);
      bool left_is_ready = computeSingleThisTime(deltaTime, left_x1000, lastState_left_x1000, left_direction, target_left);
      bool right_is_ready = computeSingleThisTime(deltaTime, right_x1000, lastState_right_x1000, right_direction, target_right);
      bool gripper_is_ready = computeSingleThisTime(deltaTime, gripper_x1000, lastState_gripper_x1000, gripper_direction, target_gripper);
      return bottom_is_ready && left_is_ready && right_is_ready && gripper_is_ready;
    }
    bool move(unsigned long time, long &bottom_x1000, long &left_x1000, long &right_x1000, long &gripper_x1000) {
      bool is_ready = computeThisTime(time, bottom_x1000, left_x1000, right_x1000, gripper_x1000);
      bottom.write(bottom_x1000 / 1000);
      left.write(left_x1000 / 1000);
      right.write(right_x1000/1000);
      gripper.setInt(gripper_x1000 / 1000 == 1);
      return is_ready;
    }
    // 计算单个电机的理想位置，返回true表示电机到位
    bool computeSingleThisTime(unsigned long deltaTime, long &angle_x1000, long &lastState_angle_x1000, short angle_direction, int target_angle) {
      if ( angle_direction == 0 ) {
        angle_x1000 = target_angle * 1000L; // 即使不需要移动，也需要传出相应的值，不能直接return
        return true;
      }
      // angle per second = angle per millisecond * 1000
      angle_x1000 = lastState_angle_x1000 + angle_direction * static_cast<signed long>(deltaTime) * speed;
      if ( angle_direction == 1 ) {
        if ( angle_x1000 >= target_angle * 1000L ) { // 注意，L不可少，谨防截断！
          angle_x1000 = target_angle * 1000L;
          return true;
        }
      }
      else if ( angle_direction == -1 ) {
        if ( angle_x1000 <= target_angle * 1000L ) {
          angle_x1000 = target_angle * 1000L;
          return true;
        }
      }
      return false;
    }
  };

  class MoveSeriesSM {
  public:
    enum class State { start, initTarget, moveTillEnd, isEnd } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES };

    static constexpr int speed = 30; // angle per second

    MoveTo moveTo{};

    size_t subIndex = 0;

    // 注意，action不得为nullptr，actionSize必须>0，不可=0！
    Rt_t run(int const action[][4], size_t const actionSize, bool reset);

  private:
    void reset_fsm() {
      state = State::start;
      subIndex = 0;
    }
  };

  class Task2SM {
  public:
    enum class State { start, idle, moving } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES };

    int const (*action)[4] = nullptr;
    size_t actionSize;

    MoveSeriesSM moveSeriesSM{};

    Rt_t run(Command command, bool reset);
  private:
    void reset_fsm() {
      state = State::start;
    }
  };

  void setup();
  bool loop();
}