// Motor.h
#include <Servo.h>
#ifndef _MOTOR_h
#define _MOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class MotorClass :Servo
{
protected:
	int _pin1;

public:
	MotorClass(int pin1);
	void motor_sturen(float snelheid, int tijd);
};

#endif
