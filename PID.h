//PID.h

#ifndef _PID_h
#define _PID_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class PIDClass
{
protected:
	double _PID; //= PID;
	float _sp;
	float _pv;

	double Kp = 1.0;
	double Ki = 0.0;
	double Kd = 0.0;

public:
	PIDClass(String name2) {};
	void init(double vorige_error, double PID, double nu_error, float sp, float pv);
	float pid(float sp, float pv);
	//	pid(float sp,float pv);

	double _vorige_error;
	double _nu_error;
	double _last_error;
};
#endif
