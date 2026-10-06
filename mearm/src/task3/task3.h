#pragma once
#include <Arduino.h>
#include <Servo.h>
#include <simple_timer.h>

extern Servo bottom;
extern Servo left;
extern Servo right;
extern Servo gripper;

// https://www.reddit.com/r/cpp_questions/comments/ripfiu/best_way_to_inherit_enum_classes 继承（组合）enum class的野生方法
#define FSM_RT_T_VALUES \
on_cpl = 0, on_going = 1

namespace task3 {
  enum class ButtonState { none = 0, first, second, third, fourth };

  enum class fsm_rt_t { FSM_RT_T_VALUES };

  class LoopMovingSM {
  public:
    enum class State { start, moveOnce, waitForReach, isEnd } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES, restore = 4 };

    unsigned long lastTime;

    // Actions
    // c++11中，类中的static constexpr需要在.cpp中提供一个类外定义，否则链接过程存在问题，在这里为了方便使用宏
#define Asize 4
    int const A[Asize][4] = {
      {90, 12, 126, 0},
      {90, 12, 126, 78},
      {54, 42, 126, 78},
      {54, 42, 126, 0}
    };

#define Bsize 1
    int const B[Bsize][4] = {
      {90, 90, 90, 0}
    };

#define Csize 2
    int const C[Csize][4] = {
      {180, 90, 90, 0},
      {90, 90, 90, 0}
    };

    int const (*actions[3])[4] = {A, B, C};
    size_t const actionSize[3] = {Asize, Bsize, Csize};
    size_t actionIndex = 0;

    // in one action
    size_t subIndex = 0;

    Rt_t run(ButtonState &clickedButton);

  private:
    void reset_fsm() {
      state = State::start;
    }
  };


  class Task3SM {
  public:
    enum class State { start, idle, loopMoving, recording, play, restore } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES };

    LoopMovingSM loopMovingSM;

    Rt_t run(ButtonState &clickedButton);

  private:
      void reset_fsm() {
        state = State::idle;
      }
  };

  void setup();

  bool loop();
}