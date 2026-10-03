#include <Arduino.h>
#include <line_buffered_input.h>

namespace task1 {
    LineBufferedInput input{Serial};

    void setup() {
        Serial.begin(9600);
        while(!Serial);
    }

    void loop() {
#define BUF_SIZE 32
        char buf[BUF_SIZE];
        if (input.getLine(buf, BUF_SIZE) != nullptr) {
            input.resetInput();

            Serial.print("received: ");
            Serial.println(buf);
        }
    }
}