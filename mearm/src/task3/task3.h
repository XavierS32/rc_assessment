#pragma once
#include <Arduino.h>
#include <Servo.h>
#include <simple_timer.h>
#include <pi_joystick.h>

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

  struct ActionRecord {
    long bottom_x1000, left_x1000, right_x1000, gripper_x1000;
    unsigned long time;
  };

  // 该状态机即使cpl后仍存在持续的内部状态，需要时需要在上层状态机手动传递信号使其彻底复位
  class LoopMovingSM {
  public:
    enum class State { start, moveOnce, waitForReach, isEnd } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES, restore = 4 };

    unsigned long lastMoveTime;

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

    Rt_t run(ButtonState &clickedButton, bool reset);

    void reset_keeping_state() {
      state = State::start;
    }

  private:
    void reset_fsm() {
      state = State::start;
      actionIndex = 0;
      subIndex = 0;
    }
  };

  class RecordSM {
  public:
    enum class State { start, record } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES, restore };

    PiJoystick bottomStick{A0, true};
    PiJoystick leftStick{A1};
    PiJoystick rightStick{A3};
    PiJoystick gripperStick{A2};
    PiJoystick::State joystickStates[4];

    unsigned long lastMoveTime, startTime;
    static constexpr int speed = 30; // angle per second

    // avr下int为16位，无法容纳到180,000
    long target_bottom_x1000, target_left_x1000, target_right_x1000, target_gripper_x1000;

    Rt_t run(ButtonState &clickedButton, bool reset, ActionRecord *(&actionRecords), size_t &actionRecordsSize, size_t &actionRecordsIndex);

    private:
      void reset_fsm() {
        digitalWrite(3, LOW);
        state = State::start;
      }
  };

  class RestoreSM {
  public:
    enum class State { start, restore, waitForReach } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES };

    unsigned long lastMoveTime;

    Rt_t run(ButtonState &clickedButton, bool reset);

    private:
      void reset_fsm() {
        state = State::start;
      }
  };

  class Task3SM {
  public:
    enum class State { start, idle, loopMoving, record, play, restore } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES };

    // 由于RecordSM和PlaySM均需要使用actionRecords，故actionRecords的生命周期应该由Task3SM来负责
    // RecordSM状态机可能会分配它，Task3SM必须确保其被合理的释放
    ActionRecord *actionRecords = nullptr;
    size_t actionRecordsSize = 0, actionRecordsIndex = 0 /* 下一个要写入的index，数值上等于已有个数 */ ;

    LoopMovingSM loopMovingSM{};
    RecordSM recordSM{};
    RestoreSM restoreSM{};

    Rt_t run(ButtonState &clickedButton, bool reset);

  private:
      void reset_fsm() {
        state = State::start;

        ButtonState tmp = ButtonState::none;
        loopMovingSM.run(tmp, true);

        if (actionRecords != nullptr) {
          free(actionRecords);
          actionRecords = nullptr;
        }
        actionRecordsSize = 0;
        actionRecordsIndex = 0;
      }
  };

  void setup();

  bool loop();

  // 替代std::equal，即使memcmp也可使用，这样在扩展以后可能更加安全
  template <typename T, size_t N>
  bool array_equal(T const (&a)[N], T const (&b)[N]) {
    for (size_t i = 0; i < N; ++i) {
      if (a[i] != b[i]) return false;
    }
    return true;
  }

  // 替代std::copy，即使memcpy也可使用，这样在扩展以后可能更加安全
  template <typename T, size_t N>
  void array_copy(T const (&src)[N], T (&dst)[N]) {
    for (size_t i = 0; i < N; ++i) {
      dst[i] = src[i];
    }
  }

  // 本来就是临时组装4个target_name_x1000与time传入，因此这里的actionRecord没必要也不能传引用
  bool addRecord(ActionRecord actionRecord, ActionRecord *(&actionRecords), size_t &actionRecordsSize, size_t &actionRecordsIndex);
}