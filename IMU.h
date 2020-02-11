// IMU.h

#ifndef _IMU_h
#define _IMU_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class IMUClass
{
protected:
	int MPU6050_read(int start, uint8_t *buffer, int size);
	int MPU6050_write(int start, const uint8_t *pData, int size);
	int MPU6050_write_reg(int reg, uint8_t data);


public:
	IMUClass(String name);

	void init();

	void Inlezen();
	float Converteren();
	void Calibreren();

	float Getpitch();
	short offset;
	short acc[3];
	short gyr[3]; //X, Y en Z waarden in de array

	float pitch;

};


#endif
