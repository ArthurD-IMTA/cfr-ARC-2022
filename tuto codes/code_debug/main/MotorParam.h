#ifndef _MOTOR_PARAMETERS_H_
#define _MOTOR_PARAMETERS_H_

// Motor macro constants
#define W_MAX 2000 // 7000 From the datasheet, keep in mind the operating range curve
#define RADIUS_WHEEL 0.036 // 36 mm meassured
#define DIST_WHEELS 0.252 // Measured 126 mm

//Port definition for motor one // LEFT
#define ANALOG_OUT_1 10
#define DIGITAL_OUT1_1 4  
#define DIGITAL_OUT2_1 5

//Port definition for motor two // RIGHT 
#define ANALOG_OUT_2 11
#define DIGITAL_OUT1_2 7
#define DIGITAL_OUT2_2 6

#endif
