#pragma once
#include <Servo.h>
namespace task1 {
  void setup(Servo &bottom, Servo &left, Servo &right, Servo &gripper);
  bool loop(Servo &bottom, Servo &left, Servo &right, Servo &gripper);
}