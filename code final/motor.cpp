#include"motor.h"

void setup_motors(int PWML, int PWMR, int dir1L, int dir2L, int dir1R, int dir2R) { //Initialize the output of the motors (card L298n) 
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);  
  pinMode(dir1L, OUTPUT);
  pinMode(dir2L, OUTPUT);
  pinMode(dir1R, OUTPUT);
  pinMode(dir2R, OUTPUT);
}

void set_speed_motor(int motor_speed, bool direction_rotation, const byte pin1, const byte pin2, const byte pinPWM, bool* sense){ 
  if (motor_speed >=0 && motor_speed <= 255){ //motor_speed needs to be between 0 and 255
    
    analogWrite(pinPWM, motor_speed);
    
    if (direction_rotation == 0){        //Rotation to move forward
      *sense = 0;
      digitalWrite(pin1,HIGH);
      digitalWrite(pin2,LOW);
    }
    else if (direction_rotation == 1){   //Rotation to move backward
      *sense = 1;
      digitalWrite(pin1,LOW);
      digitalWrite(pin2,HIGH);
    }
  }
}
void stop_motor(const byte pinPWM){
  analogWrite(pinPWM,0);
}
