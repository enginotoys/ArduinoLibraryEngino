#ifndef __ENGINOROBOTICS_H__
#define __ENGINOROBOTICS_H__

#include <Arduino.h>
#include <inttypes.h>

typedef enum portEnums
{
	MOTORA = 0,
	MOTORB,
	MOTORC,
	LED1,
	LED2,
	SENSOR1,
	SENSOR2,
	PORT_NUMBER,
	EMPTY = 0xFF
} portEnums;

enum IO_types {
  	LED = 0,   
  	IR_OBSTACLE,
	IR_LINE,
  	TOUCH,      
  	ULTRASONIC,       
	COLOUR,
	MPU6050
};

class EnginoRobotics 
{
  public:
	void getERPType();
	void getHWVersion();
	void getFWVersion();
    void setMotor(uint8_t port, uint8_t direction, uint8_t speed);
    void setRGB(uint8_t red, uint8_t green, uint8_t blue);  
	void setLed(uint8_t port, uint8_t state);
	void setLedPWM(uint8_t channel, uint8_t duty);
	void setServo(uint8_t channel, uint8_t angle);
    bool getTouch(uint8_t port);  
	bool getIR(uint8_t port);
	void getColour();
	uint16_t getColourRed();
	uint16_t getColourGreen();
	uint16_t getColourBlue();
	uint16_t getColourClear();
	void getAcc();
	void getAccX();
	void getAccY();
	void getAccZ();
	void getGyro();
	void getGyroX();
	void getGyroY();
	void getGyroZ();
	void getMPU6050Temp();
	void getMPU6050();
	void getNRF52Temp();
	uint16_t getUltrasonic();
	void configPortServo(uint8_t portA,uint8_t portB,uint8_t portC, uint8_t portD);
	void configPortLedPWM(uint8_t portA,uint8_t portB,uint8_t portC, uint8_t portD);
	void configPort(uint8_t port,uint8_t element,uint8_t state);
	void condigLineIRThreshold();
	void condigObstacleIRThreshold();
	uint8_t calibrateIRThreshold(uint8_t port);
	void StartIREnigine();
	void ActivePeripheralInit();
};

#endif