#ifndef _TRAYECTORIES_H_
#define _TRAYECTORIES_H_

#define DIV_180_PI 57.29577951

double get_dist2D(double Px, double Py);
double get_angle2D(double Px, double Py, double kx, double ky);
double calcul_s(double T, double tau, double k_T0);
void get_motor_speed_line(struct motor_angles* Angles, double k_T0,double tau, double T, double w_max);
void get_motor_speed_angle(struct motor_angles* Angles, double k_T0,double tau, double T, double w_max, int conf);

// Struct for the motor angles
struct motor_angles {
  double w1;
  double w2;
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
  double inv_tau = 1 / tau;
  if (k_T0 < tau) {
    return 1/T* k_T0 * inv_tau;
  }else if (k_T0 < T) {
    return 1/T;
  } else {
    return ((tau + T) - k_T0) * inv_tau * 1/T;
  }
}

// Returns w1 and w2 in struct at time kT0 form s(t) with param T,tau,v. 
void get_motor_speed_line(struct motor_angles* Angles, double k_T0,double tau, double T, double w_max){
   // calcul ds/dt (kT0)
  double s = calcul_s(T, tau, k_T0);
  //double aux = s * w_max;
  double aux = s;
  Angles->w1 = aux;
  Angles->w2 = aux;
}

// Returns w1 and w2 in struct at time kT0 form s(t) with param T,tau,v. 
void get_motor_speed_angle(struct motor_angles* Angles, double k_T0,double tau, double T, double w_max, int conf){
   // calcul ds/dt (kT0)
  double s = calcul_s(T, tau, k_T0);
  //double aux = s * w_max;
  double aux = s;
  Angles->w1 = aux * conf;
  Angles->w2 = -aux * conf;
}

#endif
