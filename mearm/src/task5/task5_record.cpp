#include <Arduino.h>
#include <pi_joystick.h>
#include "kinematics/kinematics.h"
#include "arm_constrain.h"
#include "task3/task3.h"
#include "task5.h"

namespace task5 {
  extern PiJoystick bottomStick;
  extern PiJoystick leftStick;
  extern PiJoystick rightStick;

  float points[5][3];
  size_t pointsIndex;
  bool record_finished = false;
}

namespace task5::record {
  constexpr int speed = 20; // angle per second

  task3::Move move;

  void setup() {
    record_finished = false;
    pointsIndex = 0;
    move.set_init(bottom.read(), left.read(), right.read(), gripper.readSetInt(), millis(), speed);
  }

  bool loop() {
    if (pointsIndex >= 5) {
      return true;
    }
    // 本来更优雅的表达应该是 < ButtonState | bool > currentEvent，但是c++11无法方便的实现union type
    ButtonState clickedButton = ButtonState::none;
    bool reset = false;
    if (Serial.available() > 0) {
      char ch = Serial.read();
      switch (ch) {
        case 1:
          clickedButton = ButtonState::first;
          break;
        case 2:
          clickedButton = ButtonState::second;
          break;
        case 3:
          clickedButton = ButtonState::third;
          break;
        case 4:
          clickedButton = ButtonState::fourth;
          break;
        case 'q':
          reset = true;
          break;
      }
    }

    if (reset) {
      return true;
    }
    else if (clickedButton == ButtonState::first
          || clickedButton == ButtonState::second
          || clickedButton == ButtonState::third) {
      task3::ignoreButtonMsg(clickedButton, "record");
    }
    else if (clickedButton == ButtonState::fourth);

    long bottom_x1000, left_x1000, right_x1000, gripper_x1000; // computeThisTime得出的角度值是理论轨迹在当前的映射,computeThisTime本身无任何副作用
    unsigned long nowTime = millis();
    move.computeThisTime(nowTime, bottom_x1000, left_x1000, right_x1000, gripper_x1000);
    bottom.write(bottom_x1000 / 1000L);
    left.write(left_x1000 / 1000L);
    right.write(right_x1000 / 1000L);
    gripper.setInt(gripper_x1000 / 1000L);

    int8_t bottomState = static_cast<int8_t>(bottomStick.stableRead());
    int8_t leftState = static_cast<int8_t>(leftStick.stableRead());
    int8_t rightState = static_cast<int8_t>(rightStick.stableRead());
    move.changeState(bottomState, leftState, rightState, 0, nowTime);

    if (clickedButton == ButtonState::fourth) { // 记录该点
      float x, y, z;
      fk_degree(bottom.read(), right.read(), left.read(), x, y, z);
      points[pointsIndex][0] = x;
      points[pointsIndex][1] = y;
      points[pointsIndex][2] = 4.4;
      if (++pointsIndex == 5) {
        // 记录结束
        // TODO... nothing?
        record_finished = true;
        return true;
      }
    } // 复位支持？…… TODO
  
    return false;
  }

  void reset() {

  }
}