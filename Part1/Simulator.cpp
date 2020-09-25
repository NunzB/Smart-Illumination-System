#include "Simulator.h"
#include <math.h>


Simulator::Simulator(float m, float b, float StaticGain, float Vcc)
{
  Simulator::m = m;
  Simulator::b = b;
  Simulator::StaticGain = StaticGain;
  Simulator::Vcc = Vcc;
  Simulator::tauConst1 = 0.043;
  Simulator::tauConst2 = 2.311572574900739e+04;
  Simulator::gainConst = 10.0152;
  Simulator::delay1 = 972 * pow(10, -6);
  Simulator::initialVoltage = 0;
  Simulator::initialTime = 0;
  Simulator::targetLuminance = 0;
  Simulator::currentGain = 0;
  Simulator::currentTau = 1;
}

float Simulator::GetTau(float lux)
{
  if (lux == 0)
    return (Simulator::tauConst1);
  else
    return (Simulator::tauConst1 * (pow(10, m * log10(lux) + b)) * 1000) / (Simulator::tauConst2 + (pow(10, m * log10(lux) + b)) * 1000);
}

float Simulator::GetGain(float lux)
{
  return  gainConst / (gainConst + pow(10, m * log10(lux) + b));
}

float Simulator::VoltageToLux(float voltage)
{
  return pow(10, (log10((Simulator::gainConst * Simulator::Vcc / voltage) - Simulator::gainConst) - b) / m);
}

void Simulator::SetTargetIlluminance(float targetLuminance, double initialTime)
{
  Simulator::targetLuminance = targetLuminance;
  Simulator::initialVoltage = Simulator::GetSystemVoltage(initialTime);
  Simulator::initialTime = initialTime;

  Simulator::currentGain = Simulator::GetGain(targetLuminance);
  Simulator::currentTau = Simulator::GetTau(targetLuminance);

  
}

float Simulator::GetSystemIlluminance(double targetTime)
{
  double time = targetTime - Simulator::delay1;
  if ((targetTime - Simulator::initialTime) - Simulator::delay1 < 0)
    time = Simulator::initialTime;
  return Simulator::VoltageToLux(Simulator::currentGain * Simulator::Vcc - ((Simulator::currentGain * Simulator::Vcc) - Simulator::initialVoltage) * exp(-(time - Simulator::initialTime) / Simulator::currentTau));
}

float Simulator::GetSystemVoltage(double targetTime)
{
  double time = targetTime - Simulator::delay1;
  if ((targetTime - Simulator::initialTime) - Simulator::delay1 < 0)
    time = Simulator::initialTime;
  return Simulator::currentGain * Simulator::Vcc - ((Simulator::currentGain * Simulator::Vcc) - Simulator::initialVoltage) * exp(-(time - Simulator::initialTime) / Simulator::currentTau);
}
