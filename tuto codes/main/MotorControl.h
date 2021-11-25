#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
#include "Trayectories.h"

// This if is temporary only until we can define it in the main
#ifndef W_MAX
#define W_MAX 14
#endif

//Port definition for motors one and two 
#define ANALOG_OUT_1 0 
#define DIGITAL_OUT1_1 9  
#define DIGITAL_OUT2_1 8

#define ANALOG_OUT_2 1 
#define DIGITAL_OUT1_2 10  
#define DIGITAL_OUT2_2 11

void setup_motors() {
  // Initialisation des variables pour le moteur
  pinMode(ANALOG_OUT_1, OUTPUT);
  pinMode(ANALOG_OUT_2, OUTPUT);  
  pinMode(DIGITAL_OUT1_1, OUTPUT);
  pinMode(DIGITAL_OUT2_1, OUTPUT);
  pinMode(DIGITAL_OUT1_2, OUTPUT);
  pinMode(DIGITAL_OUT2_2, OUTPUT);
}

// Changes motor speed based on input and outputs to pins in H bridge configuration
void change_state_motor(bool D1, bool D2, int unsigned analog_value, int DOUT1, int DOUT2, int AOUT){
  digitalWrite(DOUT1, D1);
  digitalWrite(DOUT2, D2);
  analogWrite(AOUT, analog_value);
}

void set_motor_speed(struct motor_angles* Angles){
  // Auxiliary values for the if's
  int aux1 = map(Angles->w1,0,W_MAX,0,1024);
  int aux2 = map(Angles->w2,0,W_MAX,0,1024);
  
  if (Angles->w1 > 0) {
    change_state_motor(true, false, aux1,DIGITAL_OUT1_1,DIGITAL_OUT2_1,ANALOG_OUT_1);
  }else {
    change_state_motor(false, true, aux1,DIGITAL_OUT1_1,DIGITAL_OUT2_1,ANALOG_OUT_1);
  }

  if (Angles->w2 > 0) {
    change_state_motor(true, false, aux2,DIGITAL_OUT1_2,DIGITAL_OUT2_2,ANALOG_OUT_2);
  }else {
    change_state_motor(false, true, aux2,DIGITAL_OUT1_2,DIGITAL_OUT2_2,ANALOG_OUT_2);
  }
}

#endif
