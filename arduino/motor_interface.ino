#include "Arduino.h"
#include "motor_interface.h"
#include "encoder.h"

void left_set_speed(int pwm) {
    if (pwm >= 0) {
        analogWrite(LEFT_RPWM, pwm);
        analogWrite(LEFT_LPWM, 0);
    } else if (pwm <= 0) {
        analogWrite(LEFT_RPWM, 0);
        analogWrite(LEFT_LPWM, -pwm);
    }
}

void right_set_speed(int pwm) {
    if (pwm >= 0) {
        analogWrite(RIGHT_RPWM, pwm);
        analogWrite(RIGHT_LPWM, 0);
    } else if (pwm <= 0) {
        analogWrite(RIGHT_RPWM, 0);
        analogWrite(RIGHT_LPWM, -pwm);
    }
}

inline void left_stop() {
    left_set_speed(0);
}

inline void right_stop() {
    right_set_speed(0);
}

Encoder left_encoder(LEFT_ENC_CLK, LEFT_ENC_DT, 0);
Encoder right_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT, 1);


// left_speed  [1]
// right_speed [1]
byte rx_buf[2];

// left_enc_speed  [4]
// left_enc_cnt    [4]
// right_enc_speed [4]
// right_enc_cnt   [4]
byte tx_buf[32];

uint32_t last_millis;

void setup() {


    pinMode(LEFT_LEN, OUTPUT);
    pinMode(LEFT_REN, OUTPUT);
    pinMode(RIGHT_LEN, OUTPUT);
    pinMode(RIGHT_REN, OUTPUT);

    digitalWrite(LEFT_LEN, HIGH);
    digitalWrite(LEFT_REN, HIGH);
    digitalWrite(RIGHT_LEN, HIGH);
    digitalWrite(RIGHT_REN, HIGH);

    Serial.begin(115200);
    while (!Serial); // wait for serial to open

    interrupts(); // enable interrupts

    last_millis = millis();
}


void loop() {
    last_millis = millis();

    int32_t left_speed = left_encoder.speed();
    int64_t left_cnt = left_encoder.cnt();
    int32_t right_speed = right_encoder.speed();
    int64_t right_cnt = right_encoder.cnt();
    memcpy(tx_buf, &left_speed, 4);
    memcpy(tx_buf + 4, &left_cnt, 4);
    memcpy(tx_buf + 8, &right_speed, 4);
    memcpy(tx_buf + 12, &right_cnt, 4);

    Serial.write(tx_buf, 32);
    Serial.readBytes(rx_buf, 2);

    left_set_speed((int8_t)rx_buf[0]);
    right_set_speed((int8_t)rx_buf[1]);

    while (millis() - last_millis < 100);
}
