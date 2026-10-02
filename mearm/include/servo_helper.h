#pragma once
// 一个通用的电机调试工具，支持任意个电机，可以手动调整电机的数据
// Usage:
// servo_helper::Actuator actuators[] = {
//   {servo1, "servo1_name"},
//   {servo2, "servo2_name"},
//   ...
// };
//
// auto state = servo_helper::make_state(actuators);
//
// void setup() {
//   // ...
//   servo_helper::setup(state);
// }
//
// void loop() {
//   servo_helper::loop(state);
// }
#include <Arduino.h>
#include <Servo.h>
#include <stddef.h> // 默认的avr-gcc不支持完整的标准库，如cstddef, algorithm（ https://github.com/arduino/Arduino/issues/5209 ）
// Arduino本身提供宏版的max min constrain

namespace servo_helper {
  struct Actuator {
    Servo &servo;
    char const *name;
  };

  template <size_t N>
  struct State {
    static_assert(N > 0, "servo_helper requires at least one actuator");
    Actuator (&actuators)[N];
    size_t index;
    int step;
  };

  template <size_t N>
  State<N> make_state(Actuator (&actuators)[N]) {
    return State<N>{actuators, 0, 1};
  }

  // Helper function
  template <typename T_array, typename T_print>
  void print_table_row(T_array &array, size_t N, size_t highlight, T_print doPrint) {
      // FIXME: 可能有表格对齐问题，Serial.print会返回字节数，所以理论上可以手动计算空格，但是这样徒增复杂度，而且对于非ascii字符及不可见/空白ascii字符可能有各种奇怪效果
      for (size_t i = 0; i < N; ++i) {
          if (i == highlight) {
              Serial.print("\x1b[7m");
              doPrint(array[i]);
              Serial.print("\t\x1b[0m"); // whitespace characters无法显示出颜色，所以无法高亮一整个单元格
          }
          else {
              doPrint(array[i]);
              Serial.print('\t');
          }
      }
  }

  template <size_t N>
  void print_table(State<N> &state) {
    print_table_row(state.actuators, N, state.index, [](Actuator &a){ Serial.print(a.name); });
    Serial.print("\tstep");
    Serial.println();
    print_table_row(state.actuators, N, state.index, [](Actuator &a){ Serial.print(a.servo.read(), DEC); });
    Serial.print("\t");
    Serial.print(state.step, DEC);
    Serial.println();
  }
  // Helper function END

  template <size_t N>
  void setup(State<N> &state) {
    print_table(state);
  }

  template <size_t N>
  void loop(State<N> &state) {
    if (Serial.available() > 0) {
      // do operation
      int operate = Serial.read();
      switch (operate) {
        // +(=)/-(_)：转动电机
        case '+':
        case '=':
          {
            Servo &s = state.actuators[state.index].servo;
            s.write( constrain( s.read() + state.step , 0, 180) );
          }
          break;
        case '-':
        case '_':
          {
            Servo &s = state.actuators[state.index].servo;
            s.write( constrain( s.read() - state.step , 0, 180) );
          }
          break;
        // >/<：改变操作电机
        case '>':
          if ( state.index == N - 1 ) state.index = 0;
          else  state.index++;
          break;
        case '<':
          if ( state.index == 0 ) state.index = N - 1;
          else  state.index--;
          break;
        // ./,：改变转动幅度（一次1度）；'/'/m：改变转动幅度（一次5度）
        case '.':
          state.step = min(state.step + 1, 180);
          break;
        case ',':
          state.step = max(state.step - 1, 1);
          break;
        case '/':
          state.step = min(state.step + 5, 180);
          break;
        case 'm':
          state.step = max(state.step - 5, 1);
          break;
      }
      // print status
      print_table(state);
    }
  }
}