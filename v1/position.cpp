#include"position.h"

/* Used in a previous version -- It allows us to calculate the new position of the robot without calculate dL as it is the case in the function new_pos()
void calculate_position(int* xt, int* xtt, int* yt, int* ytt, float* thetat, float* thetatt, float thetaRt, float thetaRtt, float thetaLt, float thetaLtt, int r, int d_wheels){
  if (abs(thetaRtt-thetaRt)<0.04 && abs(thetaLtt-thetaLt)<0.04){    //Minimum displacement
    *xt = *xtt;
    *yt = *ytt;
    *thetat = *thetatt;
  }
  else{
    int stock_int = *xtt;
    *xtt = *xt + int(float(r)*sin(*thetat)*(thetaRtt+thetaLtt-thetaLt-thetaRt)/2);
    *xt = stock_int;
  
    stock_int = *ytt;
    *ytt = *yt - int(float(r)*cos(*thetat)*(thetaRtt+thetaLtt-thetaLt-thetaRt)/2);
    *yt = stock_int;
  
    float stock_float = *thetatt;
    *thetatt = *thetat + float(r)*(thetaLtt+thetaRt-thetaLt-thetaRtt)/d_wheels;
    *thetat = stock_float;
  }
}*/


void new_position(int* xt, int* xtt, int* yt, int* ytt, float thetat, float dl){
  *xt = *xtt;
  *yt = *ytt;
  *xtt += dl*cos(thetat);
  *ytt += dl*sin(thetat);  
}


void send_position(int xtt, int ytt, float thetatt, float thetaRtt, float thetaLtt){
  String dataToSent = String(String(xtt) + ";" + String(ytt) + ";" + String(thetatt) + ";" + String(thetaRtt) + ";" + String(thetaLtt) + ";");
  Serial.println(dataToSent);
}

float inc_to_rad(int inc, int nbr_turn, int max_inc, int ratio){
  float thetaRad=(nbr_turn+inc/max_inc)*2*PI/ratio;
  return thetaRad;
}


// --------- Timer1 function -----------
// -------------------------------------
void init_timer1(int counter){
  noInterrupts();                 //Desactivate interruptions to avoid problems
  // We set the values of bits WGM10, WGM11, WGM12 and WGM13 to work with CTC (Clear Timer on Compare) mode
  bitClear(TCCR1A, WGM10);        // bit WGM10 = 0 (which is in register TCCR1A)
  bitClear(TCCR1A, WGM11);        // bit WGM11 = 0 (which is in register TCCR1A)
  bitSet(TCCR1B, WGM12);          // bit WGM12 = 1 (which is in register TCCR1B)
  bitClear(TCCR1B, WGM13);        // bit WGM13 = 0 (which is in register TCCR1B)
  
  //Parametrage du prédiviseur : on divise la fréquence par 1024 -> 16MHz/1024 = 15,625kHz (64micro sec de période)
  bitSet(TCCR1B,CS12);
  bitClear(TCCR1B,CS11);
  bitSet(TCCR1B,CS10);

  // On active l'interruption du timer1, qui test en permanence s'il y a égalité entre la valeur courant du timer, et la valeur stockée dans un registre de comparaison.
  bitSet(TIMSK1, OCIE1A);         // On met le bit OCIE1A à 1 (contenu dans le registre TIMSK1)

  // On met le compteur à zéro, on entre la valeur déclenchant l'interruption
  TCNT1 = 0;            // Mise du timer1 à zéro
  OCR1A = counter;        // Valeur correspondant à Te  (Te = nbr_lim /(16MHz/1024))

  //On réactive les interruptions
  interrupts();
}


// --------- PID functions -------------
// -------------------------------------
//Two functions is needed because of static values that are reused
float PID_L(float order, float actual_value, int Kp, int Ki, int Kd, float dt){
  float error = order - actual_value;
  static float Iterm = 0;
  static float last_error = 0;
  static float last_value = 0;
  Iterm += Ki*(error+last_error)*dt/2;      //With static float we can integrate the value only by making sums
  last_error = error;
  float result = Kp*error + Iterm - Kd*(actual_value-last_value)/dt;
  last_value = actual_value;
  return result;
}

float PID_A(float order, float actual_value, int Kp, int Ki, int Kd, float dt){
  float error = order - actual_value;
  static float Iterm = 0;
  static float last_error = 0;
  static float last_value = 0;
  Iterm += Ki*(error+last_error)*dt/2;
  last_error = error;
  float result = Kp*error + Iterm - Kd*(actual_value-last_value)/dt;
  last_value = actual_value;
  return result;
}
