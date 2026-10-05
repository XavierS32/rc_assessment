#include <Arduino.h>

// 新版core使用了新的api代替HardwareSerial https://forum.arduino.cc/t/compiling-errors-when-using-second-serial-in-bluepill/1456358/14
Uart Serial2( PA3 , PA2 );

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_D1, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial1.begin(9600);
  Serial2.begin(9600);
}

void loop() {
  // Serial1: computer
  // Serial2: uno
  while (Serial1.available() > 0) {
    Serial2.write((char)Serial1.read());
  }
  while (Serial2.available() > 0) {
    Serial1.write((char)Serial2.read());
  }
}