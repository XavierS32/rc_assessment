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
    static constexpr size_t A_size = 4;
    int const A[A_size][4] = {
      {90, 12, 126, 0},
      {90, 12, 126, 78},
      {54, 42, 126, 78},
      {54, 42, 126, 0}
    };

    static constexpr size_t B_size = 1;
    int const B[B_size][4] = {
      {90, 90, 90, 0}
    };

    static constexpr size_t C_size = 2;
    int const C[C_size][4] = {
      {180, 90, 90, 0},
      {90, 90, 90, 0}
    };

    static constexpr size_t Action_size = 3;
    int const (*actions[Action_size])[4] = {A, B, C};
    size_t const actionsSize[Action_size] = {A_size, B_size, C_size};
    size_t actionIndex = 0;

    // in one action
    size_t subIndex = 0;

    Rt_t run(ButtonState &clickedButton);

  private:
    void reset_fsm() {
      state = State::start;
    }
  };

  class RestoreSM {
  public:
    enum class State { start, restore, waitForReach } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES };

    unsigned long lastTime;

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
    RestoreSM restoreSM;

    Rt_t run(ButtonState &clickedButton);

  private:
      void reset_fsm() {
        state = State::start;
      }
  };

  void setup();

  bool loop();
}