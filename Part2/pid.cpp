#include "pid.h"

void pid::init(float p, float i, float d, float T, float a, float G) {      //warning: should check args
  ep = yp  = dp = 0.0f;

  kp = p;
  ki = i;
  kd = d;
  a = a;
  pid::G = G;
  T = T;
  k1 = kp = p;
  //ki = i;
  //kd = d;
  //a = a;
  k2 = kp * ki * T / 2;

  if (kd < 0.000001f)
    derivative = false; 
  else {
    derivative = true;  
    float den = kd+a*T; 
    k3=kd/den; 
    k4=kp*kd*a/den;
  }
}

float pid::calc(float ref, float y, float u) 
{   
  float e = (ref - y)/G;     
  float p = k1 * e;
  float i = ip + k2 * (e + ep);
  float d = 0;
  float u_fb = p + i + d;
  
  if (derivative)
    float d = k3 * dp - k4 * (y - yp);

  pid::CONTROL = d;
  
  u = pid::anti_windUp(u, u_fb);


  yp = y;
  ip = i;
  float dp = d;
  ep = e;

  return u;
}

float pid::anti_windUp(float u, float u_fb){// limita o duty cycle aplicado ao led entre 0 e 255, 
  if (u + u_fb < 0 | u + u_fb > 255) {      //  previne wind-up
    if (u + u_fb < 0)
      return 0;
    else
      return 255;

    p = i = d = 0;
  }
  else {
      //u += u_fb;
      return u + u_fb;
  }
}