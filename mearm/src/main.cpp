#include <Arduino.h>
#include <Servo.h>
#include <line_buffered_input.h>
#include <prefix_matcher.h>
#include "gripper/gripper.h"
#include "servo_helper/servo_helper.h"
#include "task1/task1.h"
#include "task2/task2.h"
#include "task3/task3.h"
#include "task5/task5.h"
#define ENABLE_TOOLKIT

Servo bottom;
Servo left;
Servo right;
Gripper gripper{};

servo_helper::Actuator actuators[] = {
  {bottom, "bottom"},
  {left, "left"},
  {right, "right"},
  {gripper.servo, "gripper"}/*,
  {gripper.servo2, "gripper2"}*/
};

auto state = servo_helper::make_state(actuators, Serial);

LineBufferedInput input{Serial, Serial};

typedef void (*ProgramSetup)(char const*const);

// 传入输入内容，返回true，将下一轮控制权归还shell，返回false，下一轮继续执行该函数
typedef bool (*ProgramLoop)(char const*const);

// 默认的avr-gcc不支持完整的标准库，utility缺失，故无法使用std::pair
struct ProgramFuncs {
  ProgramSetup setup;
  ProgramLoop loop;
};

void listProgram();

#ifdef ENABLE_TOOLKIT
#include "kinematics/kinematics.h"
#endif

PrefixRule<ProgramFuncs> rules[] = {
  {"list", ProgramFuncs {
    [](char const*const str) {},
    [](char const*const str) {
      listProgram();
      return true; }}},
  {"task1.1", ProgramFuncs {
    [](char const*const str) {
      servo_helper::setup(state); },
    [](char const*const str) {
      return servo_helper::loop(state); }}},
  {"task1.2", ProgramFuncs {
    [](char const*const str) {
      task1::setup(bottom, left, right, gripper); },
    [](char const*const str) {
      return task1::loop(bottom, left, right, gripper); }}},
  {"task2", ProgramFuncs {
    [](char const*const str) {
      task2::setup(); },
    [](char const*const str) {
      return task2::loop(); }}},
  {"task3", ProgramFuncs {
    [](char const*const str) {
      task3::setup(); },
    [](char const*const str) {
      return task3::loop(); }}},
  {"task5", ProgramFuncs {
    [](char const*const str) {
      task5::setup(str); },
    [](char const*const str) {
      return task5::loop(); }}}
#ifdef ENABLE_TOOLKIT
  ,{"fk", ProgramFuncs {
    [](char const*const str) {
      int a1, a2, a3;
      sscanf(str, "fk %d %d %d", &a1, &a3, &a2);
      float x, y, z;
      fk_degree_fix(a1, a2, a3, x, y, z);
      Serial.print(x, 5);
      Serial.print(' ');
      Serial.print(y, 5);
      Serial.print(' ');
      Serial.println(z, 5); },
    [](char const*const str) {
      return true; }}}
  ,{"ik", ProgramFuncs {
    [](char const*const str) {
      int x, y, z;
      sscanf(str, "ik %d %d %d", &x, &z, &y);
      int a1, a2, a3;
      if (ik_degree_fix(x, y, z, a1, a2, a3) ) {
        Serial.print(a1);
        Serial.print(' ');
        Serial.print(a2);
        Serial.print(' ');
        Serial.println(a3);
      }
      else  Serial.println("unreachable"); },
    [](char const*const str) {
      return true; }}}
#endif
};

void listProgram() {
  for (const auto &i : rules) {
    Serial.println(i.prefix);
  }
}

auto prefixMatcher = makePrefixMatcher(rules, ProgramFuncs {
  [](char const*const str) { Serial.println("program not found"); },
  [](char const*const str) { return true; }
});

ProgramFuncs currentFunc;

void setup() {
  Serial.begin(9600);
  bottom.attach(9);
  left.attach(8);
  //left.attach(5);
  right.attach(7);
  //gripper.attach(1, 6, 13); // 在此处修改舵机控制
  //gripper.attach(1, 6, 10);
  gripper.attach(0, 6);
  // 右 6 左 13

  bottom.write(90);
  left.write(90);
  right.write(90);
  gripper.set(false);

  while(!Serial);

  Serial.print("> ");
}

void loop() {
  constexpr size_t BUF_SIZE = 25;
  static enum Mode { shell, program } mode = Mode::shell;
  static char buf[BUF_SIZE];
  if ( mode == Mode::shell && input.getLine(buf, BUF_SIZE) != nullptr) {
    input.resetInput();
    currentFunc = prefixMatcher.match(buf);
    currentFunc.setup(buf);
    mode = Mode::program;
  }
  else if (mode == Mode::program) {
    mode = currentFunc.loop(buf) ? (Serial.print("> "), Mode::shell) : Mode::program;
  }
}