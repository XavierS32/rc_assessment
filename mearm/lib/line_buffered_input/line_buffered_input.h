#pragma once
// Stream.readStringUntil无法做到非阻塞的读取一整行，并在未达到一整行时暂时跳过
// Usage:
// LineBufferedInput input{Serial};

// void loop() {
// #define BUF_SIZE 32
//     static char buf[BUF_SIZE];
//     if (input.getLine(buf, BUF_SIZE) != nullptr) {
//         input.resetInput();
//         // ...
//     }
// }
#include <Arduino.h>

class LineBufferedInput {
public:
  // 绑定输入Stream，可选回显Strean
  LineBufferedInput(Stream &input) : input(input), output(nullptr) {}
  LineBufferedInput(Stream &input, Stream &output) : input(input), output(&output) {}
  // 在读取的过程中，必须保证str[0:length]不变，size>=length，除此之外，可以自由修改str和size（如realloc）
  // 非阻塞的读取一个字符串直到terminator字符出现或者buf长度达到size，如果没有达到前述条件，返回nullptr，否则返回buf
  char* getUntil(char *str, size_t size, char terminator);
  // 非阻塞的读取一个一整行字符串或者buf长度达到size，如果没有达到前述条件，返回nullptr，否则返回buf
  char* getLine(char *str, size_t size) {
    return getUntil(str, size, '\n');
  }
  // 获取已读取字符串的长度
  size_t getLength(void) {
    return length;
  }
  // IMPORTANT: 清除该轮输入（必须在重新输入前使用）
  void resetInput(void) {
    length = 0;
  }
  private:
  Stream &input;
  Stream * const output; // Stream* | nullptr
  size_t length = 0;
};