#include "IMU.h"
#include <Wire.h> //Voor de I2C communicaties
#include <math.h> //Voor de berekening van het convertern naar de hoeken

//Alle waardes die gelezen moeten worden
typedef union accel_lijst
{
	struct
	{ //Hier zijn de waardes enkel in hun eerste vorm, de 8bit registers
		uint8_t x_accel_h;
		uint8_t x_accel_l;
		uint8_t y_accel_h;
		uint8_t y_accel_l;
		uint8_t z_accel_h;
		uint8_t z_accel_l;
	} reg;
	struct
	{//hier zijn de waardes die samengevoegd zijn in een 16bit formaat
		int16_t x_accel;
		int16_t y_accel;
		int16_t z_accel;
	} waarden;
};

typedef union gyro_lijst
{
	struct
	{ //Hier zijn de waardes enkel in hun eerste vorm, de 8bit registers
		uint8_t x_gyro_h;
		uint8_t x_gyro_l;
		uint8_t y_gyro_h;
		uint8_t y_gyro_l;
		uint8_t z_gyro_h;
		uint8_t z_gyro_l;
	} reg;
	struct
	{//hier zijn de waardes die samengevoegd zijn in een 16bit formaat
		int16_t x_gyro;
		int16_t y_gyro;
		int16_t z_gyro;
	} waarden;
};

//////////////////////////////////////////
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_WHO_AM_I      0x75
//De register staan hier op een duidelijkere wijze verwoord
#define MPU6050_GYRO_CONFIG   0x1B //Hier is FS_SEL wijzigbaar
#define MPU6050_ACCEL_CONFIG  0x1C //Hier is AFS_SEL wijzigbaar
#define MPU6050_CONFIG        0x1A //
#define MPU6050_DLPF_184HZ    (bit(0))

//Alle registers dat met de metingen te maken het hebben
//registers van de accellerometer
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_ACCEL_XOUT_L  0x3C
#define MPU6050_ACCEL_YOUT_H  0x3D
#define MPU6050_ACCEL_YOUT_L  0x3E
#define MPU6050_ACCEL_ZOUT_H  0x3F
#define MPU6050_ACCEL_ZOUT_L  0x40

//#define MPU6050_TEMP_OUT_H    0x41
//#define MPU6050_TEMP_OUT_L    0x42
//registers van de gyroscoop
#define MPU6050_GYRO_XOUT_H   0x43
#define MPU6050_GYRO_XOUT_L   0x44
#define MPU6050_GYRO_YOUT_H   0x45
#define MPU6050_GYRO_YOUT_L   0x46
#define MPU6050_GYRO_ZOUT_H   0x47
#define MPU6050_GYRO_ZOUT_L   0x48
//het adres van de gyroscoop
#define MPU6050_I2C_ADDRESS   0x68

//functie => CONVERTEN ///////////////////
#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536

#define C_PI 3.14159265359//'PI' in arduino is slecht 3.14, PI is niet aanpasbaar. daarom C_PI => Constant PI
#define dt 0.01             // 10 ms sample rate!

IMUClass::IMUClass(String name) {

};

void IMUClass::init() {
	uint8_t error, c;
	//Bij het opstarten is de gyroscoop automatisch ingesteld op
	//FS_SEL=0 & AFS_SEL=0
	// Gyro op 250?/s
	// Accel op maximum 2g
	// Klok puls is op 8MHz (dit betekent geen filtering)

	error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1); //
	Serial.print(F("WHO_AM_I : "));
	Serial.print(c, HEX);
	Serial.print(F(", error = "));
	Serial.println(error, DEC);

	error = MPU6050_read(MPU6050_PWR_MGMT_1, &c, 1);
	Serial.print(F("PWR_MGMT_1 : "));
	Serial.print(c, HEX);
	Serial.print(F(", error = "));
	Serial.println(error, DEC);
	MPU6050_write_reg(MPU6050_PWR_MGMT_1, 0); //de Sleep-bit staat op 0, om de sensor te starten
											  //AFS_SEL= 0 ACCEL_CONFIG
											  //FS_SEL= 0
											  //DLPF_CFG = 1
											  //MPU6050_write_reg (MPU6050_DLPF_184HZ, 0); //filter aan zetten
	MPU6050_write_reg(MPU6050_GYRO_CONFIG, 0x08);
	MPU6050_write_reg(MPU6050_ACCEL_CONFIG, 0x08);
};

float IMUClass::Getpitch() {
	this->Inlezen();
	return this->Converteren();
}


void IMUClass::Inlezen()
{
	//Serial.println(F("begin Inlezen"));

	accel_lijst accel;
	gyro_lijst  gyro;

	MPU6050_read(0x3B, (uint8_t *)&accel, sizeof(accel)); //Accelerometer wordt gelezen
	MPU6050_read(0x43, (uint8_t *)&gyro, sizeof(gyro)); //Gyroscoop wordt gelezen

	uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap

	SWAP(accel.reg.x_accel_h, accel.reg.x_accel_l);
	SWAP(accel.reg.y_accel_h, accel.reg.y_accel_l);
	SWAP(accel.reg.z_accel_h, accel.reg.z_accel_l);
	SWAP(gyro.reg.x_gyro_h, gyro.reg.x_gyro_l);
	SWAP(gyro.reg.y_gyro_h, gyro.reg.y_gyro_l);
	SWAP(gyro.reg.z_gyro_h, gyro.reg.z_gyro_l);

	this->acc[0] = accel.waarden.x_accel;
	this->acc[1] = accel.waarden.y_accel;
	this->acc[2] = accel.waarden.z_accel;

	this->gyr[0] = gyro.waarden.x_gyro;
	this->gyr[1] = gyro.waarden.y_gyro;
	this->gyr[2] = gyro.waarden.z_gyro;

	//Serial.println(F("gedaan Inlezen"));
}
float IMUClass::Converteren()
{  
  dynamic_tilt_angle = atan((accel[1]/16384)/(accel[3]/16384));
  
  static_tilt_angle = static_tilt_angle + (gyro[0]-offset])*(1/131)*dt;
  
  pitch = 0.95 * static_tilt_angle + 0.05*dynamic_tilt_angle;
  
  return pitch;
}
//
//

// --------------------------------------------------------
  // MPU6050_read
//
int IMUClass::MPU6050_read(int start, uint8_t *buffer, int size)
{
	int i, n, error;

	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	n = Wire.write(start);
	if (n != 1)
		return (-10);

	n = Wire.endTransmission(false);    // hold the I2C-bus
	if (n != 0)
		return (n);

	// Third parameter is true: relase I2C-bus after data is read.
	Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
	i = 0;
	while (Wire.available() && i < size)
	{
		buffer[i++] = Wire.read();
	}
	if (i != size)
		return (-11);

	return (0);  // return : no error
}

// --------------------------------------------------------
// MPU6050_write
//
int IMUClass::MPU6050_write(int start, const uint8_t *pData, int size)
{
	int n, error;

	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	n = Wire.write(start);        // write the start address
	if (n != 1)
		return (-20);

	n = Wire.write(pData, size);  // write data bytes
	if (n != size)
		return (-21);

	error = Wire.endTransmission(true); // release the I2C-bus
	if (error != 0)
		return (error);

	return (0);         // return : no error
}


// --------------------------------------------------------
// MPU6050_write_reg
//
int IMUClass::MPU6050_write_reg(int reg, uint8_t data)
{
	int error;

	error = MPU6050_write(reg, &data, 1);

	return (error);
}
