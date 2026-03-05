#include "Arduino.h"

#include "config.h"

void left_set_speed(int pwm) {
    if (pwm >= 0) {
        analogWrite(LEFT_RPWM, pwm);
        analogWrite(LEFT_LPWM, 0);
    } else if (pwm <= 0) {
        analogWrite(LEFT_RPWM, 0);
        analogWrite(LEFT_LPWM, pwm);
    }
}

void right_set_speed() {
    if (pwm >= 0) {
        analogWrite(RIGHT_RPWM, pwm);
        analogWrite(RIGHT_LPWM, 0);
    } else if (pwm <= 0) {
        analogWrite(RIGHT_RPWM, 0);
        analogWrite(RIGHT_LPWM, pwm);
    }
}

inline void left_stop() {
    left_set_speed(0);
}

inline void right_stop() {
    right_set_speed(0);
}

Encoder left_encoder(LEFT_ENC_CLK, LEFT_ENC_DT);
Encoder right_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT);

void setup() {

    pinmode(LEFT_LEN, OUTPUT);
    pinmode(LEFT_REN, OUTPUT);
    pinmode(RIGHT_LEN, OUTPUT);
    pinmode(RIGHT_REN, OUTPUT);

    digitalWrite(LEFT_LEN, HIGH);
    digitalWrite(LEFT_REN, HIGH);
    digitalWrite(RIGHT_LEN, HIGH);
    digitalWrite(RIGHT_REN, HIGH);

    Serial.begin(115200);
    while (!Serial); // wait for serial to open

    interrupts(); // enable interrupts

}




void loop() {
    for (;;);
}