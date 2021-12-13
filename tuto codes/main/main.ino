#include "Arduino.h"
#include "Trayectories.h"
#include "MotorControl.h"
#include "MotorParam.h"

// Orientation macros, could be changed used to set a 0 for the angle
#define X_ORIENTATION 1
#define Y_ORIENTATION 0 


motor_angles Angles; // Main angle struct
double conf = 1; // Angle configuration will change within the script

void setup() {
  // Initialize Motor Control
  setup_motors();
  Serial.begin(9600);
}

void loop() {

  unsigned int N_points_to_traverse = 2;
  double points_to_traverse[N_points_to_traverse][2] = {{0.1,0},{0.2,0}};
  double T0 = 1e-5; // Sampling time

  double actual_position[2] = {0.2,0};
  double x_target, y_taget, tau, T, dist, angle, w;
  
  for (unsigned int k = 0; k<N_points_to_traverse; k++) {
    // Point definition
    x_target = points_to_traverse[k][0] - actual_position[0];
    y_taget = points_to_traverse[k][1] - actual_position[1];
  
    // Parameter calculation for point x_target, y_target
    dist = get_dist2D(x_target, y_taget);
    angle = get_angle2D(x_target, y_taget, X_ORIENTATION, Y_ORIENTATION);
    if (angle < 0) {
      conf = -1;
    }else{ 
      conf = 1;
    }
    
    Serial.println(dist); // DEBUG
    Serial.println(angle); // DEBUG
    
    // Point velocity profile parameters 
    tau = 0.01; // Time to ramp up / down to / from max speed
    T = 0.02; // Time to start ramping down (T >= tau)
    w = min(1/T, W_MAX);
    
    // Angle speed configuration
  
    for(int i=0;i<((tau + T) / T0);i++){
      get_motor_speed_angle(&Angles, i*T0,tau,T,w,conf);
      // Motor Control
      set_motor_speed(&Angles);
      // DEBUG Serial.println(Angles.w1); // DEBUG
      // DEBUG delay(10); //DEBUG?
      delay(T0 * 1000); // Wait for sampling time to elapse 
    }
    
    set_motor_speed_zero();
    delay(1000);// DEBUG
  
    // Line speed configuration
    tau = 0.02;
    T = 0.05; 
    w = min(1/T, W_MAX);
    
    for(int i=0;i<((tau + T) / T0);i++){
      get_motor_speed_line(&Angles, i*T0,tau,T,w);
      // Motor Control
      set_motor_speed(&Angles);
      // DEBUG Serial.println(Angles.w1);
      // DEBUG delay(10); //DEBUG?
      delay(T0 * 1000);
    }
    set_motor_speed_zero();
  
    delay(100);// DEBUG

    // Update the new position
    actual_position[0] = points_to_traverse[k][0];
    actual_position[1] = points_to_traverse[k][1];
  }
}
