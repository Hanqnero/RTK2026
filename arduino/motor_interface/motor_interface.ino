#include "motor_interface.h"
#include "encoder.h"

volatile int64_t left_cnt = 0;
volatile int64_t right_cnt = 0;

void left_set_speed(int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0) {
        analogWrite(LEFT_RPWM, pwm);
        analogWrite(LEFT_LPWM, 0);
    } else {
        analogWrite(LEFT_RPWM, 0);
        analogWrite(LEFT_LPWM, -pwm);
    }
}

void right_set_speed(int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0) {
        analogWrite(RIGHT_RPWM, pwm);
        analogWrite(RIGHT_LPWM, 0);
    } else {
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

// left_forward_pwm   [1]
// left_backward_pwm  [1]
// right_forward_pwm  [1]
// right_backward_pwm [1]
byte rx_buf[4];

// left_enc_speed  [4]
// left_enc_cnt    [4]
// right_enc_speed [4]
// right_enc_cnt   [4]
byte tx_buf[32];

uint32_t last_millis;

void setup() {


    // pinMode(LEFT_LEN, OUTPUT);
    // pinMode(LEFT_REN, OUTPUT);
    // pinMode(RIGHT_LEN, OUTPUT);
    // pinMode(RIGHT_REN, OUTPUT);

    // digitalWrite(LEFT_LEN, HIGH);
    // digitalWrite(LEFT_REN, HIGH);
    // digitalWrite(RIGHT_LEN, HIGH);
    // digitalWrite(RIGHT_REN, HIGH);

    Serial.begin(115200);
    while (!Serial); // wait for serial to open
    Serial.setTimeout(30);

    interrupts(); // enable interrupts
    // enc_start();

    last_millis = millis();
}


void loop() {
    // Serial.write(tx_buf, 32);
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');

        int left_forward = 0;
        int left_backward = 0;
        int right_forward = 0;
        int right_backward = 0;

        if (sscanf(line.c_str(), "%d %d %d %d", &left_forward, &left_backward, &right_forward, &right_backward) == 4) {
            left_forward = constrain(left_forward, 0, 255);
            left_backward = constrain(left_backward, 0, 255);
            right_forward = constrain(right_forward, 0, 255);
            right_backward = constrain(right_backward, 0, 255);

            int left_pwm = left_forward - left_backward;
            int right_pwm = right_forward - right_backward;

            left_set_speed(left_pwm);
            right_set_speed(right_pwm);
        }
    }

    while (millis() - last_millis < 100) {
    } // Ждёт чтобы каждый цикл занимал одинаковое (время как на ВПД)
    last_millis = millis();
}
