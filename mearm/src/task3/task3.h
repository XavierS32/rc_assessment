#pragma once
#include <Arduino.h>
#include <Servo.h>
#include <simple_timer.h>
#include <pi_joystick.h>
#include "button_state/button_state.h"
#include "gripper/gripper.h"
#include "fsm.h"
#include "arm_constrain.h"
#include "task2/task2.h" // 理论上同级的模块不应该相互依赖，包括下面Servo extern也不合理，但是赶时间，只能先粗糙上线了

extern Servo bottom;
extern Servo left;
extern Servo right;
extern Gripper gripper;

namespace task3 {
  enum class ButtonState : short { none = 0, first, second, third, fourth };

  enum class fsm_rt_t { FSM_RT_T_VALUES };

  constexpr size_t actionRecordsSize = 60; // 必须>=1
  struct ActionRecord {
    unsigned long time; // 变化被记录的时间
    uint8_t states; // PiJoystick::State bottom, left, right, gripper; 使用packStates和unpackState编解码 // 变化后的摇杆状态
  };
  struct Record {
    bool is_complete_record = false; // 是否完整的完成了录制
    int start_bottom, start_left, start_right, start_gripper; // 初始舵机角度
    unsigned long stopTime; // 停止的录制时间，令开始时间为0
    int speed; // angle per second
    ActionRecord actionRecords[actionRecordsSize];
    size_t actionRecordsCount;
  } extern record;

  // 其实将ActionRecord中PiJoystick::State实际上只有三个值-1 0 1，所以可以用位运算的方式压缩体积是我自己的想法
  // 但限于时间原因没有自己仔细去研究，这里的三个函数都是gpt写的
  inline uint8_t encodeState(PiJoystick::State state) {
    return static_cast<uint8_t>(
      static_cast<int>(state) + 1
    );
  }

  inline uint8_t packStates(
    PiJoystick::State b,
    PiJoystick::State l,
    PiJoystick::State r,
    PiJoystick::State g
  ) {
    return encodeState(b)
        | (encodeState(l) << 2)
        | (encodeState(r) << 4)
        | (encodeState(g) << 6);
  }

  inline short unpackState(uint8_t packed, uint8_t shift) {
    return static_cast<short>(
      static_cast<int>((packed >> shift) & 0x03u) - 1
    );
  }

  class Move {
  public:
    long lastState_bottom_x1000, lastState_left_x1000, lastState_right_x1000, lastState_gripper_x1000; // 上一次状态转变那一刻电机角度*1000
    short bottom_state = 0, left_state = 0, right_state = 0, gripper_state = 0; // -1 0 1，摇杆状态
    unsigned long lastStateChangeTime; // 上一次状态转变那一刻的时间
    int speed; // angle per second
    // 设定基础状态
    void set_init(int bottom, int left, int right, int gripper, unsigned long time, int speed) {
      lastState_bottom_x1000 = bottom * 1000L;
      lastState_left_x1000 = left * 1000L;
      lastState_right_x1000 = right * 1000L;
      lastState_gripper_x1000 = gripper * 1000L;
      lastStateChangeTime = time;
      this->speed = speed;
      bottom_state = 0;
      left_state = 0;
      right_state = 0;
      gripper_state = 0;
    }
    // 计算给定时间的理想位置
    void computeThisTime(unsigned long time, long &bottom_x1000, long &left_x1000, long &right_x1000, long &gripper_x1000) {
      unsigned long deltaTime = time - lastStateChangeTime;
      // angle per second = angle per millisecond * 1000
      bottom_x1000 = lastState_bottom_x1000 + bottom_state * static_cast<signed long>(deltaTime) * speed;
      left_x1000 = lastState_left_x1000 + left_state * static_cast<signed long>(deltaTime) * speed;
      right_x1000 = lastState_right_x1000 + right_state * static_cast<signed long>(deltaTime) * speed;
      // gripper_x1000 = lastState_gripper_x1000 + gripper_state * static_cast<signed long>(deltaTime) * speed;
      gripper_x1000 = gripper_state == 0 ? lastState_gripper_x1000
                    : gripper_state == -1 ? 0
                    : 1000L;
      bottom_x1000 = constrain(bottom_x1000, 0L, 180000L);
      left_x1000 = constrain(left_x1000, LEFT_MIN*1000L, LEFT_MAX*1000L);
      right_x1000 = constrain(right_x1000, RIGHT_MIN*1000L, RIGHT_MAX*1000L);
      // gripper_x1000 = constrain(gripper_x1000, 0L, 77000L);
    }
    // 施加一次摇杆状态变化
    void changeState(short new_bottom_state, short new_left_state, short new_right_state, short new_gripper_state, unsigned long time) {
      computeThisTime(time, lastState_bottom_x1000, lastState_left_x1000, lastState_right_x1000, lastState_gripper_x1000);
      lastStateChangeTime = time;
      bottom_state = new_bottom_state;
      left_state = new_left_state;
      right_state = new_right_state;
      gripper_state = new_gripper_state;
    }
  };

  // 该状态机即使cpl后仍存在持续的内部状态，需要时需要在上层状态机手动传递信号使其彻底复位
  class LoopMovingSM {
  public:
    enum class State { start, moveSeries } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES, restore = 4 };

    task2::MoveSeriesSM moveSeriesSM{};

    static constexpr size_t Action_size = 3;
    int const (*actions[Action_size])[4] = {task2::A, task2::B, task2::C};
    size_t const actionsSize[Action_size] = {task2::A_size, task2::B_size, task2::C_size};
    size_t actionIndex = 0;

    Rt_t run(ButtonState &clickedButton, bool reset);

    void reset_keeping_state() {
      state = State::start;
    }

  private:
    void reset_fsm() {
      state = State::start;
      actionIndex = 0;
    }
    void reset_fsm(int const action[][4], size_t const actionSize) {
      reset_fsm();
      // 按照本部分前面使用的语义，状态应该传到最深处的子状态机，由其处理，但是后面变动后引用了一个不属于task3的状态机task2::MoveSeriesSM
      // 改设计被打破。在此处，task2::MoveSeriesSM可以在任何状态被正常的复位，因此状态在LoopMoving被拦截处理，并且直接在LoopMoving的reset中传递信号复位task2::MoveSeriesSM
      moveSeriesSM.run(action, actionSize, true);
    }
  };

  class RecordSM {
  public:
    enum class State { start, record } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES, restore = 4 };

    unsigned long startTime;

    PiJoystick bottomStick{A0, true};
    PiJoystick leftStick{A1};
    PiJoystick rightStick{A3};
    PiJoystick gripperStick{A2};
    PiJoystick::State joystickStates[4];

    Move move{};

    Rt_t run(ButtonState &clickedButton, bool reset, Record &record, size_t actionRecordsSize);

  private:
    void reset_fsm() {
      digitalWrite(3, LOW);
      state = State::start;
    }
  };

  class PlaySM {
  public:
    enum class State { start, waitForInitMove, playAction } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES, restore = 4 };

    unsigned long initStartTime /*初始化设定舵机角度的时间*/, playStartTime /*开始播放的时间*/;

    size_t actionRecordsIndex;

    Move move{};

    Rt_t run(ButtonState &clickedButton, bool reset, Record &record);

  private:
    void reset_fsm() {
      state = State::start;
    }
  };

  class RestoreSM {
  public:
    enum class State { start, restore, waitForReach } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES };

    unsigned long lastMoveTime;

    Rt_t run(ButtonState &clickedButton, bool reset, bool keep_gripper = false);

    private:
      void reset_fsm() {
        state = State::start;
      }
  };

  class Task3SM {
  public:
    enum class State { start, idle, loopMoving, record, play, restore } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES };

    // record需要大量内存，故声明为全局静态变量

    LoopMovingSM loopMovingSM{};
    RecordSM recordSM{};
    PlaySM playSM{};
    RestoreSM restoreSM{};

    Rt_t run(ButtonState &clickedButton, bool reset, Record &record, size_t actionRecordsSize);

  private:
      void reset_fsm() {
        state = State::start;

        ButtonState tmp = ButtonState::none;
        loopMovingSM.run(tmp, true);
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

  // 存满无法存储当前值时返回false，否则返回true，actionRecordsSize必须>=1
  bool addRecord(ActionRecord const &actionRecord, Record &record, size_t const actionRecordsSize);
}