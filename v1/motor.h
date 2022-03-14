#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

void setup_motors(int PWML, int PWMR, int dir1L, int dir2L, int dir1R, int dir2R);
void set_speed_motor(int motor_speed, bool direction_rotation, const byte pin1, const byte pin2, const byte pinPWM, bool* sense);
void stop_motor(const byte pinPWM);

#endif
