#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
#include "MotorParam.h"
#include "Trayectories.h"

void setup_motors();
void change_state_motor(bool D1, bool D2, int analog_value, int DOUT1, int DOUT2, int AOUT);
void set_motor_speed(struct motor_angles* Angles);
void set_motor_speed_zero();

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
  // Auxiliary values for the if's. AUX2 is RIGHT MOTOR
  //int aux1 = map(Angles->w1 + W_MAX,0,2*W_MAX,8,255);
  
  //int aux1 = map(Angles->w1,0,1,100,255);
  //int aux2 = Angles->w2;
  //if (aux2 < 0){
  // aux2 *= -1;
  //}
  //aux2 = map(aux2,0,1,100,255);
  
  //aux2 = map(aux2 + W_MAX,0,2*W_MAX,8,255);

  double min_val_DEBUG = 0, val_max = 255;
  
  //int aux1 = map(Angles->w1 + W_MAX, 0, 2*W_MAX, 30, 255);
  //int aux2 = map(Angles->w2 + W_MAX, 0, 2*W_MAX, 30, 255);
  
  // Map the values from 0 to 255
  int aux1 = abs(Angles->w1) * val_max / (W_MAX);
  int aux2 = abs(Angles->w2) * val_max / (W_MAX);
  // For the smaller values give 0 since the motors dont move and are not equal at low voltage
  if (aux1 < min_val_DEBUG){
    aux1 = 0;
  }
  if (aux2 < min_val_DEBUG){
    aux2 = 0;
  }
  
  // Set the mototr pins
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

void set_motor_speed_zero(){
  change_state_motor(HIGH, LOW, 0,DIGITAL_OUT1_1,DIGITAL_OUT2_1,ANALOG_OUT_1); 
  change_state_motor(HIGH, LOW, 0,DIGITAL_OUT1_2,DIGITAL_OUT2_2,ANALOG_OUT_2);
}

#endif
