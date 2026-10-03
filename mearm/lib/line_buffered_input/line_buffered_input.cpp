#include <Arduino.h>
#include <line_buffered_input.h>

char* LineBufferedInput::getUntil(char *str, size_t size, char terminator) {
  if (str == nullptr || size == 0)    return nullptr;
  else {
    while (this->stream.available() > 0) {
      if (this->length >= size - 1) {
        str[size - 1] = '\0';
        this->length = size - 1;
        return str;
      }

      char ch = this->stream.read();
      str[this->length] = ch;
      this->length++;

      if (ch == terminator) {
        str[this->length] = '\0';
        return str;
      }
    }
    return nullptr;
  }
}