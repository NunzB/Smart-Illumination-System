#ifndef CONSENSUS_H
#define CONSENSUS_H

class Consensus
{
private:
  int ID;
  float Lref;
  float n;    //(norma de K)^2
  float m;    //n - kii^2
  float c[3];
  float z[3] = {0, 0, 0};
  float tolerance = 0.001;
  float DotProduct3(float(*a)[3], float(*b)[3]);

public:
  Consensus() {};
  float K[3];
  float d[3];
  float o;
  float d_avg[3];
  float y[3];
  float rho;
  void SetNewOperation(float lref);
  void init(int id,float Gi1, float Gi2, float Gi3, float o);
  void iterate();
  bool check_feasibility(float (*d)[3]);
  float evaluate_cost(float (*d)[3]);
};

#endif
