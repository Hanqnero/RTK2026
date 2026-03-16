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

struct __attribute__((packed)) TxPacket {
    int32_t left_speed;   // [4] encoder speed left
    int32_t left_cnt;     // [4] encoder count left
    int32_t right_speed;  // [4] encoder speed right
    int32_t right_cnt;    // [4] encoder count right
};

TxPacket tx_pkt;

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
    if (Serial.available() >= 4) {
        Serial.readBytes(rx_buf, 4);
        int left_pwm  = (int)rx_buf[0] - (int)rx_buf[1];
        int right_pwm = (int)rx_buf[2] - (int)rx_buf[3];
        left_set_speed(left_pwm);
        right_set_speed(right_pwm);
    }

    tx_pkt.left_speed  = (int32_t)left_enc.speed();
    tx_pkt.left_cnt    = (int32_t)left_enc.cnt();
    tx_pkt.right_speed = (int32_t)right_enc.speed();
    tx_pkt.right_cnt   = (int32_t)right_enc.cnt();
    Serial.write((byte*)&tx_pkt, sizeof(tx_pkt));

    while (millis() - last_millis < 100) {
    } // Ждёт чтобы каждый цикл занимал одинаковое (время как на ВПД)
    last_millis = millis();
}
