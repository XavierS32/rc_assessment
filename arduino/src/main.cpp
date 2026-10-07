#include <Arduino.h>
#include <buttons.h>

// 新版core使用了新的api代替HardwareSerial https://forum.arduino.cc/t/compiling-errors-when-using-second-serial-in-bluepill/1456358/14
Uart Serial2(PA3 , PA2);

PullUpButton b1(PB10);
PullUpButton b2(PB11);
PullUpButton b3(PB12);
PullUpButton b4(PB13);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_D1, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial1.begin(9600);
  Serial2.begin(9600);
  b1.setup();
  b2.setup();
  b3.setup();
  b4.setup();
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

  if ( b1.clicked() ) {
    static int i = 0;
    Serial.print("[arduino] button 1: ");
    Serial.println(i++, DEC);
    Serial2.print((char)1); // print数字会默认转化为字符形式，需要显式转化为char类型，来发送控制字符
  }
  if ( b2.clicked() ) {
    static int i = 0;
    Serial.print("[arduino] button 2: ");
    Serial.println(i++, DEC);
    Serial2.print((char)2);
  }
  if ( b3.clicked() ) {
    static int i = 0;
    Serial.print("[arduino] button 3: ");
    Serial.println(i++, DEC);
    Serial2.print((char)3);
  }
  if ( b4.clicked() ) {
    static int i = 0;
    Serial.print("[arduino] button 4: ");
    Serial.println(i++, DEC);
    Serial2.print((char)4);
  }
}