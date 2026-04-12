#include "motor.h"

void motors_begin() {
    pinMode(LEFT_RPWM,  OUTPUT);
    pinMode(LEFT_LPWM,  OUTPUT);
    pinMode(RIGHT_RPWM, OUTPUT);
    pinMode(RIGHT_LPWM, OUTPUT);
    motor_left_set(0);
    motor_right_set(0);
}

static void _set(uint8_t pin_fwd, uint8_t pin_bwd, int pwm, int sign) {
    pwm = constrain(pwm * sign, -255, 255);
    if (pwm > 0) {
        analogWrite(pin_fwd, pwm);
        analogWrite(pin_bwd, 0);
    } else if (pwm < 0) {
        analogWrite(pin_fwd, 0);
        analogWrite(pin_bwd, -pwm);
    } else {
        analogWrite(pin_fwd, 0);
        analogWrite(pin_bwd, 0);
    }
}

void motor_left_set(int pwm)  { _set(LEFT_RPWM,  LEFT_LPWM,  pwm, LEFT_MOTOR_SIGN);  }
void motor_right_set(int pwm) { _set(RIGHT_RPWM, RIGHT_LPWM, pwm, RIGHT_MOTOR_SIGN); }

