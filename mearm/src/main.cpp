#include <Arduino.h>
#include <Servo.h>
#include "servo_helper/servo_helper.h"
#include "task1/task1.h"
#include <line_buffered_input.h>
#include <prefix_matcher.h>

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
   {"task3", ProgramFuncs {
    [](char const*const str) {
      task3::setup(); },
    [](char const*const str) {
      return task3::loop(); }}}
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
  right.attach(7);
  gripper.attach(6);

  bottom.write(90);
  left.write(90);
  right.write(90);
  gripper.write(0);

  while(!Serial);

  Serial.print("> ");
}

void loop() {
  constexpr size_t BUF_SIZE = 32;
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