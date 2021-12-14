#ifndef _TRAYECTORIES_H_
#define _TRAYECTORIES_H_
#include "MotorParam.h"

#define DIV_180_PI 57.29577951

double get_dist2D(double Px, double Py);
double get_angle2D(double Px, double Py, double kx, double ky);
double calcul_s(double T, double tau, double k_T0);
void get_motor_speed_line(struct motor_angles* Angles, double k_T0,double tau, double T);
void get_motor_speed_angle(struct motor_angles* Angles, double k_T0,double tau, double T, int conf);
void calculate_motor_final_angles(struct motor_angles* Angles, double dist, double angle);

// Struct for the motor angles
struct motor_angles {
  double w1;
  double w2;
  double theta_length_final;
  double theta_angle_final;
};

// Both points should be given in the same unit
double get_dist2D(double Px, double Py) {
  return sqrt(Px*Px + Py*Py);
}

// Gets point and an orientation and calculates the angle between them in -180 to 180
double get_angle2D(double Px, double Py, double kx, double ky){
  double norm_p = sqrt(Px*Px + Py*Py);
  double norm_k = sqrt(kx*kx + ky*ky);
  double result = acos((Px*kx + Py*ky) / (norm_p * norm_k)) * DIV_180_PI;
  // Converts to degree
  if (Py < ky) {
    result = -result;
  }
  return result;
}

// Calculates s prime in time kT0 with param T, tau and v
double calcul_s(double T, double tau, double k_T0) {
  
  if (k_T0 < tau) {
    return k_T0 * (1 / (tau*T));
  }else if (k_T0 < T) {
    return 1/T;
  } else if (k_T0 <= T + tau){
    return ((tau + T) - k_T0) * (1 / (tau*T));
  } else{
    return 0;  
  }
}

// Returns w1 and w2 in struct at time kT0 form s(t) with param T,tau,v. 
void get_motor_speed_line(struct motor_angles* Angles, double k_T0,double tau, double T){
   // calcul ds/dt (kT0)
  double s = calcul_s(T, tau, k_T0);
  Angles->w1 = s * Angles->theta_length_final;
  Angles->w2 = s * Angles->theta_length_final;
}

// Returns w1 and w2 in struct at time kT0 form s(t) with param T,tau,v. 
void get_motor_speed_angle(struct motor_angles* Angles, double k_T0,double tau, double T, int conf){
   // calcul ds/dt (kT0)
  double s = calcul_s(T, tau, k_T0);
  Angles->w1 = s * conf * Angles->theta_angle_final;
  Angles->w2 = -s * conf * Angles->theta_angle_final;
}

void calculate_motor_final_angles(struct motor_angles* Angles, double dist, double angle){
  Angles->theta_length_final = dist / RADIUS_WHEEL;
  Angles->theta_angle_final = angle * DIST_WHEELS / (2*RADIUS_WHEEL);
}


#endif
