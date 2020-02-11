#include "Motor.h"
#include <Servo.h>

MotorClass::MotorClass(int pin1) :Servo()
{
	this->_pin1 = pin1;
	pinMode(this->_pin1, OUTPUT);

	attach(pin1);
}

void MotorClass::motor_sturen(float snelheid, int tijd)
{
	write(snelheid);
	delay(tijd);
	write(90);// af zetten
};
