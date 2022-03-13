/* CREDITS
 * 
 * 
 * 
 * USEFULL RESSOURCES
 * Geometric model of the robot : https://lucidar.me/fr/mechanics/geometric-model-for-differential-wheeled-mobile-robot/
 * Manage timers in arduino : https://www.locoduino.org/spip.php?article84
 * 
 * CONTRIBUTORS (feel free to contact us)
 * Arthur DIDIER : arthur.didier@imt-atlantique.net
 */
 
#include "constants.h"
#include "motor.h"
#include "trajectory.h"
#include "position.h"



void setup() {
  //Affichage
  Serial.begin(9600);
  init_capteurs();
  setup_motors(pwmL, pwmR, dir1_L, dir2_L, dir1_R, dir2_R);
  init_timer1(counter_max_timer1);
  

}

void loop() {
  //calculate_position(&x_t, &x_tt, &y_t, &y_tt, &theta_t, &theta_tt, thetaR_t, thetaR_tt, thetaL_t, thetaL_tt, radius_wheel, dist_wheels);
  //send_position(x_tt, y_tt, theta_tt, thetaR_tt, thetaL_tt);

  /*
  static bool setp = 0;
  static int i = 0;

  
  if (i<6){   
    Serial.print(" x : ");Serial.println(x_setpoints[i]);
    Serial.print(" y : ");Serial.println(y_setpoints[i]);
    Serial.print(" theta : ");Serial.println(theta_setpoints[i]);
    setp = 1;
  }

  
  if (setp){
    xc = x_setpoints[i];
    yc = y_setpoints[i];
    thetao = theta_setpoints[i];
    move_to_setpoint();
    setp = 0;
    i++;
  }
  */

  
  xc = 0;
  yc = 0;
  thetac += PI/8;
  Serial.print("Consigne : ");Serial.println(thetac,5);
  //move_to_setpoint();
  read_regulator_setting();
  delay(4000);
  Serial.print("Mesure fin de trajectoire : ");Serial.println(theta_tt,5);
  delay(2000);
  Serial.println("\n\n");

  //send_position(x_tt, y_tt, theta_tt, thetaR_tt, thetaL_tt);

  //Serial.print("Right : ");Serial.println(thetaR_tt);
  //Serial.print("Left : ");Serial.println(thetaL_tt);
  //delay(2000);
  
}


void init_capteurs(){ //Interruptions to detect the rotation of the wheels
  //Pin mode
  pinMode(pinR,INPUT_PULLUP);
  pinMode(pinL,INPUT_PULLUP);
  //Interruptions
  attachInterrupt(digitalPinToInterrupt(pinR), turnR, RISING);
  attachInterrupt(digitalPinToInterrupt(pinL), turnL, RISING);
}

void turnR(){     //For the rigth wheel
  static byte nbr_pulse = 0;
  if (senseR){    //We know in which sens the wheel turn with the way we control it
    posR_inc++;   //Add incrementation
    nbr_pulse++;
    /* //Previous version
    if (posR_inc==max_inc_coder){
      posR_inc = 0;
      nbr_turnR++;
    }
    */
  }
  else{
    posR_inc--;
    nbr_pulse++;
    /* //Previous version (can be used if nbr_pulse_max >> Nbr increments per revolution)
    if (posR_inc==-max_inc_coder){
        posR_inc = 0;
        nbr_turnR--;
      }
      */
  }

  if (nbr_pulse == nbr_pulse_max){    //each nbr_pulse_max we calculate the new position, it allows us to have the minimum length detection desired
    new_pos();
    nbr_pulse = 0;
    posR_inc = 0;
  }
}

void turnL(){     //For the rigth wheel
  static byte nbr_pulse = 0;
  if (senseL){    //We know in which sens the wheel turn with the way we control it
    posL_inc++;   //Add incrementation
    nbr_pulse++;
    /* //Previous version
    if (posR_inc==max_inc_coder){
      posR_inc = 0;
      nbr_turnR++;
    }
    */
  }
  else{
    posL_inc--;
    nbr_pulse++;
    /* //Previous version (can be used if nbr_pulse_max >> Nbr increments per revolution)
    if (posR_inc==-max_inc_coder){
        posR_inc = 0;
        nbr_turnR--;
      }
      */
  }

  if (nbr_pulse == nbr_pulse_max){    //each nbr_pulse_max we calculate the new position, it allows us to have the minimum length detection desired
    new_pos();
    nbr_pulse = 0;
    posL_inc = 0;
  }
}


//------PID functions--------------------------------------------------------
//---------------------------------------------------------------------------
float saturation(float u){
  float usat = u; 
  if (u>alim_motor){usat = alim_motor; } 
  else{ 
    if (u<-alim_motor){usat = -alim_motor;}
  }
  return usat;
}

void new_pos(){                 //This functions is 
  thetaR_t = thetaR_tt;         //Stock previous position of the right wheel
  thetaL_t = thetaL_tt;         //Same for the left wheel
  thetaR_tt = inc_to_rad(posR_inc, nbr_turnR, max_inc_coder, reduc_ratio);              //Calculates the angle of the right wheel
  thetaL_tt = inc_to_rad(posL_inc, nbr_turnL, max_inc_coder, reduc_ratio);              //same thing for the left one

  
  //Polar control
  L_t = L_tt;
  float dL = -radius_wheel*(thetaR_tt-thetaR_t + thetaL_tt-thetaL_t)/2;
  L_tt += dL;

  new_position(&x_t, &x_tt, &y_t, &y_tt, theta_tt+PI/2, dL);

  theta_t = theta_tt;
  theta_tt += radius_wheel*(thetaL_tt-thetaL_t - thetaR_tt+thetaR_t)/dist_wheels;
}

ISR(TIMER1_COMPA_vect){
  static int i = 0;
  /*
  thetaR_t = thetaR_tt;         //Stock previous position of the right wheel
  thetaL_t = thetaL_tt;         //Same for the left wheel
  thetaR_tt = inc_to_rad(posR_inc, nbr_turnR, max_inc_coder, reduc_ratio);              //Calculates the angle of the right wheel
  thetaL_tt = inc_to_rad(posL_inc, nbr_turnL, max_inc_coder, reduc_ratio);              //same thing for the left one

  
  //Polar control
  L_t = L_tt;
  float dL = -radius_wheel*(thetaR_tt-thetaR_t + thetaL_tt-thetaL_t)/2;
  L_tt += dL;

  new_position(&x_t, &x_tt, &y_t, &y_tt, theta_tt+PI/2, dL);

  theta_t = theta_tt;
  theta_tt += radius_wheel*(thetaL_tt-thetaL_t - thetaR_tt+thetaR_t)/dist_wheels;
  */
  //Error calculations + PID
  float u_length = PID_L(Lc, L_tt, Kp_length, Ki_length, Kd_length, Te);
  float u_angle = PID_A(thetac, theta_tt, Kp_rot, Ki_rot, Kd_rot, Te);

  float u_rightW = inv_radius_wheel*(-u_length-dist_wheels*u_angle/2);
  float u_leftW = inv_radius_wheel*(-u_length+dist_wheels*u_angle/2);

  
  u_rightW = saturation(u_rightW);              //We compare to the maximum voltage
  u_leftW = saturation(u_leftW);

  if (abs(u_leftW) < min_voltage_L){  stop_motor(pwmL);  }
  else {  
    set_speed_motor(map(abs(u_leftW), min_voltage_L, alim_motor, minPWM_L, maxPWM_L), u_leftW>0, dir1_L, dir2_L, pwmL, &senseL);  
  }

  if (abs(u_rightW) < min_voltage_R){  stop_motor(pwmR);  }
  else {  
    set_speed_motor(map(abs(u_rightW), min_voltage_R, alim_motor, minPWM_R, maxPWM_R), u_rightW>0, dir1_R, dir2_R, pwmR, &senseR);  
  }  
  
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------


void move_to_setpoint(){
  // --- TRAJECTORY
  // variables needed
  float xi = x_tt;
  float yi = y_tt;
  float t = 0;

  //First move of the trajectory = rotation to be aligned with the direction
  float vi = 0;         //(theta_tt-theta_t)/Te;
  float qi = theta_tt;

  Serial.println("Start 3s");
  Serial.println("Orientation");
  Serial.print("Goal theta direc : ");Serial.println(angle_setpoint(qi,follow_orientation(xi,yi,xc,yc,qi)),5);
  delay(1500);

  bool end_setpoint = 0;
  while (end_setpoint==0){
    thetac = setpoint_generation(amax_rot, vmax_rot, vi, qi, angle_setpoint(qi,follow_orientation(xi,yi,xc,yc,qi)), t);
    if (thetac == angle_setpoint(qi,follow_orientation(xi,yi,xc,yc,qi))){
      end_setpoint = 1;
    }
    t += T_ech;
    delay(T_ech*1000);
    //Serial.print("thetac :");Serial.println(thetac);
  }

  send_position(x_tt, y_tt, theta_tt, thetaR_tt, thetaL_tt);

  Serial.println("Move forward");
  delay(1500);

  t=0;
  end_setpoint = 0;
  
  vi = 0; //(L_tt-L_t)/Te;
  
  qi = L_tt;
  xi = x_tt;
  yi = y_tt;
  Serial.print("Goal Lc : ");Serial.println(L_tt + length_setpoint(xi,yi,xc,yc),5);
  while (end_setpoint==0){
    Lc = setpoint_generation(amax, vmax, vi, qi, qi + length_setpoint(xi,yi,xc,yc), t);
    if (Lc == qi + length_setpoint(xi,yi,xc,yc)){
      end_setpoint = 1;
    }
    t += T_ech;
    delay(T_ech*1000);
    //Serial.print("Lc :");Serial.println(Lc);

  }
  send_position(x_tt, y_tt, theta_tt, thetaR_tt, thetaL_tt);
  
  Serial.println("Final orientation");
  Serial.print("Theta end : ");Serial.println(angle_setpoint(theta_tt,thetao),5);

  delay(1500);

  
  t = 0;
  end_setpoint = 0;
  
  vi = 0; //(theta_tt-theta_t)/Te;
  qi = theta_tt;
  
  while (end_setpoint==0){
    thetac = setpoint_generation(amax_rot, vmax_rot, vi, qi, angle_setpoint(qi,thetao), t);
    if (thetac == angle_setpoint(qi,thetao)){
      end_setpoint = 1;
    }
    t += T_ech;
    delay(T_ech*1000);
    //Serial.print("thetac :");Serial.println(thetac);
  }
  delay(1500);
}




// --- Functions that are only usefull to debug or configure the system
void read_regulator_setting(){
  String inputs ="";
  if (Serial.available() > 0) {
    inputs = Serial.readString();
    int s_index=-1;
    int e_index=-1;
    for (int i =0; i<inputs.length();i++){
      if (inputs[i]=='s'&&s_index==-1){
        s_index = i;
      }
      if (inputs[i]=='e'&&s_index!=-1&&e_index==-1){
        e_index = i;
      }
    }
    float* coef[6] = {&Kp_length, &Ki_length, &Kd_length, &Kp_rot, &Ki_rot, &Kd_rot};
    
    int index = s_index+1;
    
    for (int i=0;i<6;i++){  
      String str_value;
      while (inputs[index] != ';'&&index<inputs.length()){
        str_value += String(inputs[index]);
        index ++;                
      }
      index ++;
      
      *coef[i] = str_value.toFloat();
    }
  }
}
