#include <Servo.h>
#include "IMU.h"
#include "Motor.h"
#include <Wire.h> //Voor de I2C communicaties
#include <math.h> //Voor de berekening van het converteren naar de hoeken

//////////////servo MOTOR//////
//Rechter servo
#define rechts 8

//Linker servo
#define links 9

Servo motorRechts;
Servo motorLinks;

IMUClass IMU("IMU");

//servo variabelen
#define snelheid_para 1
#define laagste_error 0.15
float snelheid;

//////////////GYROSCOOP/////////
int error;

float begin_pitch;
/////////////PID////////////////
double error_pid_vorig;
double error_pid = 0.00;

double maxpid = 0;
//---------------------------------//
///////////////SETUP/////////////////
void setup()
{
	Serial.begin(9600);
	Wire.begin();
	IMU.init();
	motorRechts.attach(rechts);
	motorLinks.attach(links);
	offset = IMU.Getpitch();
	Serial.println(begin_pitch);
}
/////////////////LOOP/////////////////
void loop()
{
	motor(IMU.Getpitch() - begin_pitch);
}

void motor_klok(float snelheid, int tijd)
{
	if (snelheid > 0)
	{
		motorLinks.write(90 - snelheid);
		motorRechts.write(90 + snelheid);

		delay(tijd);

		motorRechts.write(90);
		motorLinks.write(90);
	};
}
void motor_tegen_klok(float snelheid, int tijd)
{
	if (snelheid > 0)
	{
		motorLinks.write(90 + snelheid);
		motorRechts.write(90 - snelheid);

		delay(tijd);

		motorRechts.write(90);
		motorLinks.write(90);
	}
}
void motor(float pitch)
{
	error_pid_vorig = error_pid;
	error_pid = 0.0 - pitch;

	int stappen;

	if (abs(error_pid) < laagste_error) //PID uit
	{
		stappen = 0;
		error_pid = 0;
		error_pid_vorig = 0;

		motorLinks.write(90);
		motorRechts.write(90);
	}

	if (abs(error_pid) > laagste_error)// PID aan
	{
		double P_err = error_pid;
		double I_err = error_pid + error_pid_vorig;
		double D_err = error_pid - error_pid_vorig;

		double Kp = 1.0; //lage 
		double Ki = 2.0;
		double Kd = 0.0;

		double pid = (Kp*P_err + Ki*I_err + Kd*D_err) * 10;
		maxpid = 30;
		//Serial.println(String(pitch*10) + " " + String(pid));
		pid = abs(pid);
		if (pitch < 0.0) {
			stappen = map(pid, 0, maxpid, 0, 30);

			motorLinks.write(83 - stappen);
			motorRechts.write(95 + stappen);
		};
		if (pitch == 0.0) {
			motorLinks.write(90);
			motorRechts.write(90);
		};
		if (pitch > 0.0) {
			stappen = -1 * map(pid, 0, maxpid, 0, 30);

			motorLinks.write(95 - stappen);
			motorRechts.write(83 + stappen);
		};

	};
}
