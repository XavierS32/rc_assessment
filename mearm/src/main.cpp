#include <Arduino.h>
#include <Servo.h>
#include "servo_helper/servo_helper.h"

Servo bottom;
Servo left;
Servo right;
Servo gripper;

servo_helper::Actuator actuators[] = {
  {bottom, "bottom"},
  {left, "left"},
  {right, "right"},
  {gripper, "gripper"}
};

auto state = servo_helper::make_state(actuators, Serial);

void setup() {
  Serial.begin(9600);
  bottom.attach(9);
  left.attach(8);
  right.attach(7);
  gripper.attach(6);

  gripper.write(0);

  while(!Serial);

  servo_helper::setup(state);
}

void loop() {
  servo_helper::loop(state);
}