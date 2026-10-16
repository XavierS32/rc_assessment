#pragma once
#include <Arduino.h>
#include <Servo.h>
class Gripper {
public:
  Servo servo/*默认电机*/, servo2/*额外的电机*/;
  int8_t mode = 0; // 状态：0，李的爪子（单电机）；1，林的爪子（双电机）；2，我的爪子（单电机）
  // mn_xx -> mode{n}下，爪子打开与夹紧的角度
  uint8_t m0_close = 120, m0_open = 20;
  uint8_t m1_close = 50/*TODO*/, m1_open = 130;
  uint8_t m2_close = 77, m2_open = 0;

  void attach(int8_t mode, uint8_t servoPin) {
    servo.attach(servoPin);
    servo2.detach();
    this->mode = mode;
  }

  void attach(int8_t mode, uint8_t servoPin, uint8_t servo2Pin) {
    servo.attach(servoPin);
    servo2.attach(servo2Pin);
    this->mode = mode;
  }

  void write(uint8_t value) {
    value = constrain(value, 0, 180);
    servo.write(value);
    if (mode == 1) {
      servo2.write(180 - value);
    }
  }

  // false打开，true夹紧
  void set(bool close) {
    if (mode == 0) {
      write( close ? m0_close : m0_open );
    }
    else if (mode == 1) {
      write( close ? m1_close : m1_open );
    }
    else if (mode == 2) {
      write( close ? m2_close : m2_open );
    }
  }

  // 0（或所有非1的数）打开，1夹紧
  void setInt(uint8_t close) {
    set(close == 1);
  }

  uint8_t read() {
    return servo.read();
  }

  // 无奈之举，只要没有完全达到张开的角度，就认为已经夹紧了
  bool readSet() {
    if (mode == 0) {
      return servo.read() > m0_open;
    }
    else if (mode == 1) {
      return servo.read() </*TODO*/ m1_open;
    }
    else if (mode == 2) {
      return servo.read() > m2_open;
    }
    return true; // 消除编译器的警告
  }

  // 0打开，1夹紧
  uint8_t readSetInt() {
    return readSet() ? 1 : 0;
  }
};