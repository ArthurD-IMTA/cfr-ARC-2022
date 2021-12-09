#include "Arduino.h"
#include "Trayectories.h"
#include "MotorControl.h"
#include "MotorParam.h"

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
  double T0 = 0.001;
  double tau = 0.1;
  double T = 0.11; // (T >= tau)
  
  // Parameter calculation for point x_target, y_target
  double dist = get_dist2D(x_target, y_taget);
  double angle = get_angle2D(x_target, y_taget, X_ORIENTATION, Y_ORIENTATION);
  double w = min(1/T, W_MAX);
  
  
  if (angle < 0) {
    conf = -1;
  }else{ 
    conf = 1;
  }
  
  // Angle speed configuration
  int k_lim = (tau + T) / T0;
  
  for(int i=0;i<k_lim;i++){
    get_motor_speed_angle(&Angles, i*T0,tau,T,w,conf);
    // Motor Control
    set_motor_speed(&Angles);
    Serial.println(Angles.w1);
    delay(10); //DEBUG?
  }
  
  set_motor_speed_zero();
  delay(100);// DEBUG

  // Line speed configuration
  tau = 10;
  T = 10.11; 
  k_lim = (tau + T) / T0;
  
  for(int i=0;i<k_lim;i++){
    get_motor_speed_line(&Angles, i*T0,tau,T,w);
    // Motor Control
    set_motor_speed(&Angles);
    Serial.println(Angles.w1);
    delay(10); //DEBUG?
  }
  set_motor_speed_zero();

  delay(100);// DEBUG

}
