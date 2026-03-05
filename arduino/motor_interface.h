#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include "stdint.h" // int64_t

#define LEFT_LEN 1
#define LEFT_REN 1
#define RIGHT_LEN 1
#define RIGHT_REN 1

#define LEFT_LPWM 1
#define LEFT_RPWM 1
#define RIGHT_LPWM 1
#define RIGHT_RPWM 1

#define LEFT_ENC_CLK 1
#define LEFT_ENC_DT 1
#define RIGHT_ENC_CLK 1
#define RIGHT_ENC_DT 1


void left_stop();
void right_stop();

void left_set_speed(int pwm);
void right_set_speed(int pwm);


volatile int64_t left_cnt;
volatile int64_t right_cnt;


#endif