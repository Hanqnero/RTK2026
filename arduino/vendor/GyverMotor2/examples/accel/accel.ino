#include <Arduino.h>
#include <GyverMotor2.h>

// GyverMotor2<GM2::DIR_DIR> motor(5, 6);
// GyverMotor2<GM2::DIR_PWM> motor(5, 6);
GyverMotor2<GM2::DIR_PWM_INV> motor(5, 6);
// GyverMotor2<GM2::PWM_PWM_SPEED> motor(5, 6);
// GyverMotor2<GM2::PWM_PWM_POWER> motor(5, 6);
// GyverMotor2<GM2::DIR_DIR_PWM> motor(5, 6, 7);

void setup() {
    // установка ускорения
    motor.setAccel(100);
}

void loop() {
    // вызов тикера
    motor.tick();

    // скорость будет применяться плавно
    int speed = map(analogRead(0), 0, 1023, -255, 255);
    motor.runSpeed(speed);
}