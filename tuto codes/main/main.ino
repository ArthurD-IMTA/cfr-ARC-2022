#include "Arduino.h"
#include "Trayectories.h"
#include "MotorControl.h"

// Motor macro constants
#define W_MAX 14
#define RADIUS_WHEEL 0.01
#define DIST_WHEELS 0.32

// Orientation macros, could be changed
#define Y_ORIENTATION 0 
#define X_ORIENTATION 1

motor_angles Angles; // Main angle struct
double conf = 1; // Angle configuration will change within the script

void setup() {
  // Initialize Motor Control
  setup_motors();
  Serial.begin(9600);
}

void loop() {
  
  // Point definition
  double x_target = 1;
  double y_taget = 1;

  // Point velocity profile parameters where; w = alpha * w_max and tau = beta * T
  double T0 = 1;
  double alpha = 0.5; // Between 0 and 1
  double beta = 0.1; // Between 0 and 1

  // Parameter calculation for point x_target, y_target
  double dist = get_dist2D(x_target, y_taget);
  double angle = get_angle2D(x_target, y_taget, X_ORIENTATION, Y_ORIENTATION);
  double w = alpha*W_MAX;
  if (angle < 0) {
    conf = -1;
  }else{ 
    conf = 1;
  }

  // Main Loop for going to point in (dist, angle)
  double T = calculate_T(W_MAX,RADIUS_WHEEL,beta,alpha, dist);
  double tau = beta*T;
  int k_lim = (2*tau + T) / T0;
  
  for(int i=0;i<k_lim;i++){
    get_motor_speed_line(&Angles, i*T0,tau,T,w);
    // Motor Control
    set_motor_speed(&Angles);
    Serial.println(Angles.w1);
    delay(10); 
  }
  
  T = calculate_T(W_MAX,RADIUS_WHEEL,beta,alpha, DIST_WHEELS*angle);
  tau = beta*T;
  k_lim = (2*tau + T) / T0;
 
  for(int i=0;i<k_lim;i++){
    get_motor_speed_angle(&Angles, i*T0,tau,T,w,conf);
    // Motor Control
    set_motor_speed(&Angles);
    Serial.println(Angles.w1);
    delay(10); 
  }
  
}
