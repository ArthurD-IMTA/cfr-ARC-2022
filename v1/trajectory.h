#ifndef TRAJECTOIRE_H
#define TRAJECTOIRE_H
#include"Arduino.h"

int sign(float x);
float modulo(float x, float y);
float setpoint_generation(float amax, float vmax, float vi, float yi, float yf, float t);
float angle_setpoint(float theta, float theta_setpoint);
float length_setpoint(float xi, float yi, float xf, float yf);
float follow_orientation(float xi, float yi, float xf, float yf, float thetai);


#endif
