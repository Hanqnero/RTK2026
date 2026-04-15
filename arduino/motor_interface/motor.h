#pragma once
#include <Arduino.h>
#include "config.h"

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Управление мотором через H-мост BTS7960
//
// pwm > 0 → вперёд (RPWM)
// pwm < 0 → назад  (LPWM)
// pwm = 0 → стоп   (оба 0)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void motors_begin();
void motor_left_set(int pwm);
void motor_right_set(int pwm);
