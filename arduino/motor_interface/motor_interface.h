#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include "stdint.h"

// #define LEFT_LEN 1
// #define LEFT_REN 1
// #define RIGHT_LEN 1
// #define RIGHT_REN 1
#define LEFT_RPWM 11
#define LEFT_LPWM 10
#define RIGHT_RPWM 8
#define RIGHT_LPWM 9


#define LEFT_ENC_CLK 2
#define LEFT_ENC_DT 3
#define RIGHT_ENC_CLK 18
#define RIGHT_ENC_DT 19


void left_stop();
void right_stop();

void left_set_speed(int pwm);
void right_set_speed(int pwm);

#endif
