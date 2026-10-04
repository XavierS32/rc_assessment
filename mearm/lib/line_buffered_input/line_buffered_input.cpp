#include <Arduino.h>
#include <line_buffered_input.h>

char* LineBufferedInput::getUntil(char *str, size_t size, char terminator) {
  if (str == nullptr || size == 0)    return nullptr;
  else {
    while (input.available() > 0) {
      char ch = input.peek();
      // 本来这里想实现的是从一个流中读入，但是从键盘输入tty到成为流中间，仍然有一层，处理一些控制字符
      // 这个抽象层在这里缺失（没空写了），导致不得不在这一层弥补一些，但是这本质上是不合理的
      // 最终效果的具体表现就是，对控制字符的处理仍然很不完善，如果输入一些控制字符，不会触发对应的效果，而是会原样存储，导致占了空间又有诡异的视觉效果
      // 另外，也让这个类本质上变得不通用，不再像fgets那样可以处理通用的fd，而是成为专门处理tty到读取
      // 同样的，另外一个应该由键盘到tty层做的事情，回显，在这里也包办了
      if (ch == 0x7F) input.read(); // DEL（由于没有光标跳转功能，光标始终在末尾，DEL不会删除任何东西（不用操作读入内容，也不用回显任何内容））
      else if (ch == 0x08) { // 退格
        input.read();
        if (length == 0);
        else {
          if (output != nullptr)  output->write("\b \b");
          length--;
        }
      }
      else if ( 0x20 <= ch && ch <= 0x7E ) { // 只处理可打印字符
        if (length >= size - 1) {
          str[size - 1] = '\0';
          length = size - 1;
          return str;
        }
        else {
          input.read();
          if (output != nullptr)  output->write(ch);

          str[length] = ch;
          length++;
    
          if (ch == terminator || length == size - 1) {
            str[length] = '\0';
            return str;
          }
        }
      }
      else  input.read(); // 丢弃其他控制字符
    }
    return nullptr;
  }
}