#ifndef SIMULATOR_H
#define SIMULATOR_H

class Simulator 
{
private:
	float m;
	float b;
	float StaticGain;
	float Vcc;
	float tauConst1; 		//Tau(x) = (TauConst1*(10^(m*log10(x)+b))*10^(3))/(TauConst2+(10^(m*log10(x)+b))*10^(3))
	float tauConst2;
	float gainConst;		//Gain(x) = (GainConst/(GainConst+10^(m*log10(x)+b)))	[v/Vcc]
	float delay1;
	float initialVoltage;
	double initialTime;
	float targetLuminance;
	float currentTau;
	float currentGain;

	float GetGain(float lux);
	float GetTau(float lux);
	float VoltageToLux(float voltage);
	   
public:
	float TargetIlluminance() const { return Simulator::targetLuminance; };

	Simulator(float m, float b, float StaticGain, float Vcc);

	void SetTargetIlluminance(float targetLuminance, double initialTime);
	float GetSystemIlluminance(double targetTime);
	float GetSystemVoltage(double targetTime);
};

#endif
