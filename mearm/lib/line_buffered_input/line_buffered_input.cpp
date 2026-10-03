#include <Arduino.h>
#include <line_buffered_input.h>

char* LineBufferedInput::getUntil(char *str, size_t size, char terminator) {
  if (str == nullptr || size == 0)    return nullptr;
  else {
    while (stream.available() > 0) {
      if (length >= size - 1) {
        str[size - 1] = '\0';
        length = size - 1;
        return str;
      }

      char ch = stream.read();
      str[length] = ch;
      length++;

      if (ch == terminator) {
        str[length] = '\0';
        return str;
      }
    }
    return nullptr;
  }
}