#include"trajectory.h"

int sign(float x){
  if (x<0){
    return -1;
  }
  else if (x>0){
    return 1;
  }
  return 0;
}

float modulo(float x, float y){
  return x - int(x/y)*y;
}

float setpoint_generation(float amax, float vmax, float vi, float qi, float qf, float t){ 
//t must be greater than 0
  if (qi == qf){
    return qi;
  }
  else{
    static float tau = 0;
    static float T = 0;
  
    amax = sign(qf-qi)*amax;
    vmax = sign(qf-qi)*vmax;
  
    tau = abs((vmax-vi)/amax);
    T = (qf+amax*tau*tau)/vmax-tau;
  
    if (T>tau){
      if (t<=tau){
        return qi+vi*t+0.5*amax*t*t;
      }
      else if (t>tau&&t<=T){
        return vmax*t-0.5*amax*tau*tau;
      }
      else if (t>T && t<= T+tau){
        return vmax*t-0.5*amax*((t-T)*(t-T)+tau*tau);
      }
      else {
        return qf;
      }
    }
    else if (T<=tau){
      float tau2 = (-vi+sqrt(vi*vi+amax*(qf-qi)))/amax;
  
      if(tau2<0){
        tau2 = (-vi-sqrt(vi*vi+amax*(qf-qi)))/amax;
      }
      if (t<=tau2){
        return vi*t+0.5*amax*t*t+qi;
      }
      else if (t>tau2 && t<=2*tau2){
        return vi*t+amax*(tau2*tau2-0.5*(2*tau2-t)*(2*tau2-t))+qi;
      }
      else {
        return qf;
      }
    }
  }
}


float angle_setpoint(float theta, float theta_setpoint){ //this function allows us to minimize the rotation of the robot including the fact that theta could be greater than 2pi
  theta_setpoint = modulo(theta_setpoint,2*PI);
  float theta_mod = modulo(theta,2*PI);
  if (abs(theta_setpoint-theta_mod)<=PI){
    return theta+(theta_setpoint-theta_mod);
  }
  else if ((theta_mod-theta_setpoint)>PI){
    return theta+2*PI-theta_mod+theta_setpoint;
  }
  else {
    return theta-2*PI-theta_mod+theta_setpoint;
  }
}


float length_setpoint(float xi, float yi, float xf, float yf){
  return sqrt((xf-xi)*(xf-xi)+(yf-yi)*(yf-yi));
}


float follow_orientation(float xi, float yi, float xf, float yf, float thetai){
  if ((xf-xi)>0){
    return atan((yf-yi)/(xf-xi))-PI/2;          //We remove PI/2 to have the rigth angle
  }
  else if((xf-xi)<0){
    return PI/2+atan((yf-yi)/(xf-xi));
  }

  if (yf==yi){
    return thetai;
  }
  return (1-sign(yf-yi))*PI/2;
}
