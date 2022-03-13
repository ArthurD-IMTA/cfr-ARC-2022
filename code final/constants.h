#ifndef CONSTANTS_H
#define CONSTANTS_H

// --- Mecanical constants
const byte reduc_ratio = 36;          //Reduction ratio of motors
const byte radius_wheel = 36;         //Radius of wheels in MILLIMETERS
const float inv_radius_wheel  = 0.027778;
const byte dist_wheels = 252;         //distance between wheels in MILLIMETERS
const float inv_dist_wheels = 0.003968;



// --- Electronic constants
const int max_inc_coder = 500;          //Number of pulses per revolution for encoders (wheel rigth and left are supposed to be the same)
const byte alim_motor = 16;             //Motor power supply
const int minus_alim_motor = -16;
const float min_voltage_L = 0.01;       //Minimum voltage from which we activate the left motor
const float min_voltage_R = 0.01;       //Same for the right motor
const byte minPWM_L = 5;                //Define the minimum pwm value for the left wheel (at least 0)
const byte minPWM_R = 5;                //same for the right wheel
int maxPWM_L = 150;                     //Define the minimum pwm value for the left wheel (up to 255)
int maxPWM_R = 150;                     //same for the right wheel
const byte nbr_pulse_max = 40;

// --- Pins connection
//Encoder
const byte pinR = 2;             //Channel A of the encoder of the rigth wheel
const byte pinL = 3;             //Channel A of the encoder of the left wheel
//Motor control
const byte dir1_L = 7;           //Direction control of the left wheel...
const byte dir2_L = 8;           //...second pin
const byte dir1_R = 12;          //Same for the right wheel
const byte dir2_R = 13;
const byte pwmL = 5;              //Speed control of the left wheel (PWM)
const byte pwmR = 6;             //Speed control of the right wheel (PWM)




// --- VARIABLES
//Sensor
int posR_inc = 0;               //Count pulses for the right encoder
int nbr_turnR = 0;              //Count the number of turn for the right encoder

int posL_inc = 0;               //Same for the left wheel
int nbr_turnL = 0;              //...

//Position
float thetaR_t = 0;             //Previous position of the right wheel (in radians)
float thetaR_tt = 0;            //Actual position of the right wheel (in radians)
float thetaL_t = 0;             //Previous position of the right wheel (in radians)
float thetaL_tt = 0;            //Actual position of the right wheel (in radians)

int x_t = 0;                    //Previous X position of the robot compared to the starting position (in millimeters)
int x_tt = 0;                   //Actual X position of the robot ...
float dx_tt = 0;
int y_t = 0;                    //Same with Y axis...
int y_tt = 0;                   //All in millimeters
float theta_t = 0;              //Previous orientation theta of the robot
float theta_tt = 0;             //Actual orientation ...
float L_t = 0;                  //Length traveled previous
float L_tt = 0;                 //Length traveled actually

//Consigne
int xc = x_tt;
int yc = y_tt;
float thetao;                   //Orientation that we 
float Lc = L_tt;                //Setpoint initialized at the current position (to have regulator error equal to zero)
float thetac = theta_tt;

//Control
bool senseR = 0;                //Rotation sense of the rigth wheel
bool senseL = 0;                //Rotation sense of the left wheel


// --- Regulator
//s0;0.000;0.000;2.7;2.3;0.4;e
float Kp_rot =2.7;
float Ki_rot = 2.3;
float Kd_rot = 0.4;
float Kp_length = 0;
float Ki_length = 0;
float Kd_length = 0;
const float Te = 0.1;            //Sampling time (in sec)

const int counter_max_timer1 = 15625 * Te; //Corresponds to the number of pulses that is needed to have an interrupt every Te seconds

// --- Trajectory
const float T_ech = 0.1;        //Setpoint change frequency
 
float amax_rot = 0.2;           //rad/s²
float vmax_rot = 1;             //rad/s

float amax = 100;               //mm/s²
float vmax = 600;               //mm/s



#endif
