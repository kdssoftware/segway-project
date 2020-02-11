//
//

#include "PID.h"
#include <math.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
#include "PID.h"

void PIDClass::init(
	double vorige_error,
	double nu_error,
	double PID ,
	float sp,
	float pv
	)
{
	this->_vorige_error = vorige_error;
	this->_nu_error = nu_error;
	this->_PID = PID;
	this->_sp = sp;
	this->_pv = pv;
}

float PIDClass::pid(float sp, float pv)
{
	//waarde nog afstemmen met de error
	_vorige_error = _nu_error;
	_nu_error = sp - pv;

	double P_err = _nu_error;
	double I_err = _nu_error + _vorige_error;
	double D_err = _nu_error - _vorige_error;

	return (Kp*P_err + Ki*I_err + Kd*D_err);
}
