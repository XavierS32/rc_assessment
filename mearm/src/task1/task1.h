#pragma once
#include <Servo.h>
#include "gripper/gripper.h"
namespace task1 {
  void setup(Servo &bottom, Servo &left, Servo &right, Gripper &gripper);
  bool loop(Servo &bottom, Servo &left, Servo &right, Gripper &gripper);
}