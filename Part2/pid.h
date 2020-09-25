#ifndef PID_H 
#define PID_H

class pid 
{
private:  
  bool derivative; 
  float kp, ki, kd, T, a; 
  float k1, k2, k3, k4;
  float e, p, yp, i, d, dp, ep, ip;
  float G, u_fb, ref, y;
  float u;
  float anti_windUp(float u, float u_fb);

public:
  float CONTROL;
  void init(float kp, float ki, float kd, float T, float a, float G); 
  float calc(float ref , float y, float u);
};
#endif //PID_H
