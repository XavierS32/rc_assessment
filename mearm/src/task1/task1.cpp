#include <Arduino.h>
#include <Servo.h>
#include <stdio.h>
#include <line_buffered_input.h>
#include <limits.h>

namespace task1 {
  LineBufferedInput input{Serial, Serial};
  
  int speed = 1; // angle per second
  int target_bottom, target_left, target_right;
  unsigned long lastMillis;
  int restDeltaAngle = 0;

  template <typename T>
  constexpr bool between(T x, T low, T high) {
    return (low <= x && x <= high);
  }

  void setArmTarget(char const*const str) {
    int input_bottom = -1, input_left = -1, input_right = -1;
    sscanf(str, "x%d,y%d,z%d", &input_bottom, &input_left, &input_right);
    if (between(input_bottom, 0, 180))  target_bottom = input_bottom;
    if (between(input_left, 0, 180))  target_left = input_left;
    if (between(input_right, 0, 180))  target_right = input_right;
    Serial.print("set target to move to: bottom:");   Serial.print(target_bottom, DEC);
    Serial.print(" left:");                           Serial.print(target_left, DEC);
    Serial.print(" right:");                          Serial.print(target_right, DEC);
    Serial.println();
  }

  // 根据目标角与当前角的大小决定移动方向，并确保移动停止在目标角
  inline void applyAngle(Servo &servo, int target_angle, int deltaAngle) {
    int now_angle = servo.read(); // 并非真正当前角度，只是上次write的角度
    now_angle += constrain(target_angle - now_angle, -deltaAngle, deltaAngle);
    servo.write(now_angle);
    // char buf[1024];
    // sprintf(buf, "move to %d, deltaAngle %d\n", now_angle, deltaAngle);
    // Serial.println(buf);
  }

  // 要求并没有要求转动行为的实现方式，估选择一种最简单的方式，所有电机的移动速度一致，而非到达时间一致
  // 有时候移动速度过小，电机最少一次只能设置移动一度，故当当前时机对应移动角度不足一度时，直接跳过这次移动，积攒到后面一起移动
  void moveArmOnce(Servo &bottom, Servo &left, Servo &right, Servo &gripper) {
    unsigned long nowMillis = millis();
    unsigned long pastMillis = nowMillis - lastMillis;
    lastMillis = nowMillis;

    restDeltaAngle += speed * pastMillis;
    int nowDeltaAngle = restDeltaAngle / 1000;
    if (nowDeltaAngle > 0) {
      restDeltaAngle %= 1000;
      applyAngle(bottom, target_bottom, nowDeltaAngle);
      applyAngle(left, target_left, nowDeltaAngle);
      applyAngle(right, target_right, nowDeltaAngle);
      // Serial.println("start move");
    }
  }

  void setup(Servo &bottom, Servo &left, Servo &right, Servo &gripper) {
    target_bottom = bottom.read();
    target_left = left.read();
    target_right = right.read();
    lastMillis = millis();
  }

  void loop(Servo &bottom, Servo &left, Servo &right, Servo &gripper) {
    static enum class Mode { Char, Line } mode = Mode::Char;
    constexpr size_t BUF_SIZE = 32;
    char buf[BUF_SIZE];
    if ( mode == Mode::Char && Serial.available() > 0) {
      char ch = Serial.peek();
      switch (ch) {
        case 'o':
        case 'O':
          Serial.read();
          gripper.write(0);
          Serial.println("gripper opens");
          break;
        case 's':
        case 'S':
          Serial.read();
          gripper.write(76);
          Serial.println("gripper closes");
          break;
        case 'h':
        case 'H':
          Serial.read();
          speed = min(speed + 2, 5400); // 实际电机的最快速度未知
          Serial.print("speed increased: ");
          Serial.print(speed, DEC);
          Serial.println();
          break;
        case 'l':
        case 'L':
          Serial.read();
          speed = max(speed - 2, 1);
          Serial.print("speed decreased: ");
          Serial.print(speed, DEC);
          Serial.println();
          break;
        case 'x':
          mode = Mode::Line;
          Serial.print("> ");
          break;
        default:
          Serial.read();
          break;
      }
    }
    // FIXME: corner case: 如果line buffered的上一条指令太长被截断，剩余部分会被自动带到下一条指令
    // 正常shell的语义应该是丢弃超出部分，但是正常shell通常有行缓冲，按下回车键前指令不会被直接发送，人也不知道会被截断
    // 而由于在这里抽象层的丢失，以及人类是边打字边同步看回显的，所以可以直接发现截断运行的现象
    // 故在这个简单任务里问题不大，就暂不处理了
    else if ( mode == Mode::Line ) {
      if (input.getLine(buf, BUF_SIZE) != nullptr) {
        input.resetInput();

        Serial.println();

        setArmTarget(buf);

        mode = Mode::Char;
      }
    }

    moveArmOnce(bottom, left, right, gripper);
  }
}