#pragma once
#include <Servo.h>
#include "fsm.h"
#include "gripper/gripper.h"
#include "button_state/button_state.h"
#include "task3/task3.h"

extern Servo bottom;
extern Servo left;
extern Servo right;
extern Gripper gripper;


namespace task5 {
  inline float map_UL_to_F(unsigned long value, unsigned long fromLow, unsigned long fromHigh, float toLow, float toHigh) {
    return static_cast<float>(value - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) + toLow;
  }

  using SetupFunc = void (*) ();
  using LoopFunc = bool (*) (unsigned long);
  using ResetFunc = void (*) ();

  class ControlSM {
  public:
    enum class State { start, move, pause, waitBeforeStop, stop } state = State::start;
    enum class Rt_t { FSM_RT_T_VALUES };

    unsigned long baseTime, baseTimeLength, readyForStop;

    task3::RestoreSM restoreSM{};

    Rt_t run(ButtonState const &clickedButton, bool reset, SetupFunc setupFunc, LoopFunc LoopFunc, ResetFunc resetFunc);
  private:
    void reset_fsm() {
      state = State::start;
    }
  };

  using SubProgramSetup = void (*) (char const* str);
  using SubProgramLoop = bool (*) ();
  struct SubProgramFuncs {
    SubProgramSetup setup;
    SubProgramLoop loop;
  };

  void setup(char const*const str);
  bool loop();

  enum class MoveState { ongoing, finished, unreachable };

  MoveState moveDirect(float const (&pStart)[3], float const (&pEnd)[3], unsigned long const time, unsigned long const relativeTime);
  namespace direct {
    void setup();
    bool loop(unsigned long relativeTime);
    void reset();
  }

  namespace shape {
    void setup();
    bool loop(unsigned long relativeTime);
    void reset();
  }

  extern float points[5][3];
  extern bool record_finished;

  namespace record {
    void setup();
    bool loop();
    void reset();
  }

  namespace polyline {
    void setup();
    bool loop(unsigned long relativeTime);
    void reset();
  }

  namespace curve {
    void setup();
    bool loop(unsigned long relativeTime);
    void reset();
  }

  bool controlSMLoop();

  using MoveFunc = MoveState (*) (const float (&pStart)[3], const float (&pEnd)[3], unsigned long relativeTime, size_t index);
  void multiMoveSetup(unsigned long relativeTime);
  bool multiMoveLoop(unsigned long relativeTime, float points[][3], size_t pointsSize, size_t &pointsIndex, MoveFunc moveFunc, bool pause = false, unsigned long pauseTime = 700);
}