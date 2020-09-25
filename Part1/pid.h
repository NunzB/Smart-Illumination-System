#ifndef PID_H 
#define PID_H
class pid 
{
    bool derivative; 
    float kp, ki, kd, T, a; 
    float k1, k2, k3, k4; 
    float ep, yp, ip, dp;
    
  public:
    void init(float kp, float ki, float kd, float T, float a); 
    //void print();
    float calc(float ref , float y);
};
#endif //PID_H
