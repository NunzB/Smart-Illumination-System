#include "pid.h"


void pid::init(float p, float i, float d, float _T, float _a) {      //warning: should check args
  ep = yp = ip = dp = 0.0f;
  k1 = kp = p;
  ki = i;
  kd = d;
  T = _T;
  a = _a;
  k2 = kp * ki * T / 2;
  /*if (kd < 0.000001f)
    derivative = false;
  else {*/
    derivative = true;
    float den = kd + a * T;
    k3 = kd / den;
    k4 = kp * kd * a / den;
  //}
}
float pid::calc(float ref, float y) { //warning: code not optimized
  float erro = ref - y;
  float e = erro / 2.0121;
  float p = k1 * e;
  float i = ip + k2 * (e + ep);
  float d = 0;

  if (derivative)
    float d = k3 * dp - k4 * (y - yp);
  yp = y;
  ip = i;
  float dp = d;
  ep = e;
  return p + i + d;
}
/*void pid::print(){
  cout << “ kp: “ << kp;
  cout << “ ki: “ << ki;
  cout << “ kd: “ << kd;
  cout << “ T: “ << T;
  cout << “ a: ” << a << endl;
  }*/
