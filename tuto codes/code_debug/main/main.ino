#include "Arduino.h"
#include "Trayectories.h"
#include "MotorControl.h"
#include "MotorParam.h"

motor_angles Angles; // Main angle struct
double conf = 1; // Angle configuration will change within the script

void setup() {
  // Initialize Motor Control
  setup_motors();
  Serial.begin(9600);
}

void loop() {

  double T0 = 1e-5; // Sampling time
  double tau, T, dist, angle;
  
  // Parameter calculation for point x_target, y_target
  dist = 0.1;
  angle = 90;
  int N_turns = 4;
  int N_lines = 2;

  calculate_motor_final_angles(&Angles, dist, angle);
  
  // Angle velocity profile parameters 
  tau = 1; // Time to ramp up / down to / from max achieved speed
  T = 2; // Time to start ramping down (T >= tau)
  
  // Condition to not exceed W_MAX with small values of T
  if (Angles.theta_angle_final / T > W_MAX) {
    T = Angles.theta_angle_final / W_MAX;
  }
  for(int k=0; k<N_turns; k++){   
    for(int i=0;i<((tau + T) / T0);i++){
      // Motor Control
      get_motor_speed_angle(&Angles, i*T0,tau,T,conf);
      set_motor_speed(&Angles);
      delay(T0 * 1000); // Wait for sampling time to elapse 
    }
    set_motor_speed_zero();
    delay(500);
  }
  
  // Line speed configuration
  tau = 2; // Time to ramp up / down to / from max achieved speed
  T = 6; // Time to start ramping down (T >= tau)
  // Condition to not exceed W_MAX with small values of T
  if (Angles.theta_length_final / T > W_MAX) {
    T = Angles.theta_length_final / W_MAX;
  }

  for(int k=0; k<N_lines; k++) {
    for(int i=0;i<((tau + T) / T0);i++){
      // Motor Control
      get_motor_speed_line(&Angles, i*T0,tau,T);
      set_motor_speed(&Angles);
      delay(T0 * 1000);
    }
    set_motor_speed_zero();
    delay(500);
  }
}
