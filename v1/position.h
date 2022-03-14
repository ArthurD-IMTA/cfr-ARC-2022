#ifndef POSITION_H
#define POSITION_H

#include <Arduino.h>
#include<math.h>


//fonctions
void calculate_position(int* xt, int* xtt, int* yt, int* ytt, float* thetat, float* thetatt, float thetaRt, float thetaRtt, float thetaLt, float thetaLtt, int r, int d_wheels);
void send_position(int xtt, int ytt, float thetatt, float thetaRtt, float thetaLtt);                              //Function that allow us to communicate the position of the robot to python program
float inc_to_rad(int inc, int nbr_turn, int max_inc, int ratio);
void init_timer1(int counter);
float PID_L(float order, float actual_value, int Kp, int Ki, int Kd, float dt);
float PID_A(float order, float actual_value, int Kp, int Ki, int Kd, float dt);
void new_position(int* xt, int* xtt, int* yt, int* ytt, float thetat, float dl);

#endif
