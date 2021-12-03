#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
#include "MotorParam.h"
#include "Trayectories.h"

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
void change_state_motor(bool D1, bool D2, int analog_value, int DOUT1, int DOUT2, int AOUT){   
  analogWrite(AOUT, analog_value); 
  digitalWrite(DOUT1, D1);
  digitalWrite(DOUT2, D2);  
}

void set_motor_speed(struct motor_angles* Angles){
  // Auxiliary values for the if's
  int aux1 = map(Angles->w1 + W_MAX,0,2*W_MAX,0,100);
  
  int aux2 = Angles->w2;
  if (aux2 < 0){
   aux2 *= -1;
  }
  
  aux2 = map(aux2,0,W_MAX,0,255);
  
  if (Angles->w1 > 0) {
    change_state_motor(true, false, aux1,DIGITAL_OUT1_1,DIGITAL_OUT2_1,ANALOG_OUT_1);
  }else {
     change_state_motor(false, true, aux1,DIGITAL_OUT1_1,DIGITAL_OUT2_1,ANALOG_OUT_1);
  }
  if (Angles->w2 > 0) {
    change_state_motor(HIGH, LOW, aux2,DIGITAL_OUT1_2,DIGITAL_OUT2_2,ANALOG_OUT_2);
  }else {
    change_state_motor(LOW, HIGH, aux2,DIGITAL_OUT1_2,DIGITAL_OUT2_2,ANALOG_OUT_2);
  }
}

void set_motor_speed_zero(){
  change_state_motor(HIGH, LOW, 0,DIGITAL_OUT1_1,DIGITAL_OUT2_1,ANALOG_OUT_1); 
  change_state_motor(HIGH, LOW, 0,DIGITAL_OUT1_2,DIGITAL_OUT2_2,ANALOG_OUT_2);
}

#endif
