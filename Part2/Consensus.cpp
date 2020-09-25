#include "Consensus.h"
#include <math.h>


void Consensus::init(int id, float Gi1, float Gi2, float Gi3, float o)
{
  Consensus::ID = id;
  Consensus::K[0] = Gi1;
  Consensus::K[1] = Gi2;
  Consensus::K[2] = Gi3;
  Consensus::rho = 0.7;
  Consensus::c[0] = 0;
  Consensus::c[1] = 0;
  Consensus::c[2] = 0;
  Consensus::c[id] = 1;
  Consensus::SetNewOperation(0);
  Consensus::n = Consensus::DotProduct3(&(Consensus::K), &(Consensus::K));
  Consensus::m = Consensus::n - pow(Consensus::K[id], 2);
  Consensus::Lref = 0;
  Consensus::o = o;
}

void Consensus::SetNewOperation(float lref)
{
  Consensus::Lref = lref;
  Consensus::y[0] = 0;
  Consensus::y[1] = 0;
  Consensus::y[2] = 0;
  Consensus::d[0] = 0;
  Consensus::d[1] = 0;
  Consensus::d[2] = 0;
  Consensus::d_avg[0] = 0;
  Consensus::d_avg[1] = 0;
  Consensus::d_avg[2] = 0;
}

void Consensus::iterate()
{
  float d_best[3] = { -1, -1, -1 };

  for (int i = 0; i < 3; i++)
  {
    Consensus::z[i] = Consensus::rho*Consensus::d_avg[i] - Consensus::y[i] - Consensus::c[i]; //Definition of z used in the unconstrained and constrained cases
    d_best[i] = (1.0 / Consensus::rho)*Consensus::z[i];
  }

  if (Consensus::check_feasibility(&d_best))  // if the unconstrained solution exists, then it is the optimal solution
    return d_best;

  float cost_best = 1000000;
  float bondary_cost = 0;

  float dotProdZK = Consensus::DotProduct3(&(Consensus::z), &(Consensus::K));

  /*float d_bl[3];
  float d_b0[3];
  float d_b1[3];
  float d_l0[3];
  float d_l1[3];*/

  float d_b[5][3];

  // constrained cases
  for (int i = 0; i < 3; i++)
  {
    d_b[0][i] = (1.0 / Consensus::rho)*Consensus::z[i] - Consensus::K[i] / Consensus::n*(Consensus::o - Consensus::Lref + (1 / Consensus::rho*dotProdZK));
    d_b[1][i] = (1.0 / Consensus::rho)*Consensus::z[i];
    d_b[2][i] = d_b[1][i];
    d_b[3][i] = (1.0 / Consensus::rho)*Consensus::z[i] - (1.0 / Consensus::m)*Consensus::K[i] * (Consensus::o - Consensus::Lref) + (1.0 / (Consensus::rho*Consensus::m))*Consensus::K[i] * (Consensus::K[Consensus::ID] * Consensus::z[Consensus::ID] - dotProdZK);
    d_b[4][i] = (1.0 / Consensus::rho)*Consensus::z[i] - (1.0 / Consensus::m)*Consensus::K[i] * (Consensus::o - Consensus::Lref + 100.0 * Consensus::K[Consensus::ID]) + (1.0 / (Consensus::rho*Consensus::m))*Consensus::K[i] * (Consensus::K[Consensus::ID] * Consensus::z[Consensus::ID] - dotProdZK);
  }

  d_b[1][Consensus::ID] = 0;
  d_b[2][Consensus::ID] = 100;
  d_b[3][Consensus::ID] = 0;
  d_b[4][Consensus::ID] = 100;

  for (int j = 0; j < 5; j++)
  {
    if (Consensus::check_feasibility(&d_b[j]))
    {
      if ((bondary_cost = Consensus::evaluate_cost(&d_b[j])) < cost_best)
      {
        cost_best = bondary_cost;
        for (int k = 0; k < 3; k++)
        {
          d_best[k] = d_b[j][k];
          Consensus::d[k] = d_best[k];
        }
      }
    }
  }

    return;
}

bool Consensus::check_feasibility(float (*d)[3])
{
  if ((*d)[Consensus::ID] < (0 - Consensus::tolerance))
    return false;

  if ((*d)[Consensus::ID] > (100 + Consensus::tolerance))
    return false;

  float left = Consensus::DotProduct3(d, &(Consensus::K));
  float right = (Consensus::Lref - Consensus::o - Consensus::tolerance);
  if (Consensus::DotProduct3(d, &(Consensus::K)) < (Consensus::Lref - Consensus::o - Consensus::tolerance))
    return false;

  return true;
}

float Consensus::evaluate_cost(float (*d)[3]) {
  float aux[3] = { (*d)[0] - Consensus::d_avg[0], (*d)[1] - Consensus::d_avg[1], (*d)[2] - Consensus::d_avg[2] };
  float e = Consensus::DotProduct3(&aux, &aux);
  return Consensus::DotProduct3(&(Consensus::c), d) + Consensus::DotProduct3(&(Consensus::y), &(Consensus::d_avg)) + Consensus::rho / 2.0 * Consensus::DotProduct3(&aux, &aux);
}

float Consensus::DotProduct3(float(*a)[3], float(*b)[3])
{
  float result = 0;
  for (int i = 0; i < 3; i++)
  {
    result += (*a)[i] * (*b)[i];
  }
  return result;
}
