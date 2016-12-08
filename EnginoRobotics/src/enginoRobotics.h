#ifndef __ENGINOROBOTICS_H__
#define __ENGINOROBOTICS_H__

#include <Arduino.h>
#include <inttypes.h>

#define SPI_DELAY			50
#define CS 					9

#define CLOCKWISE			0x10
#define ANTICLOCKWISE		0x20
#define BRAKE				0x30
#define OFF					0x40

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

typedef enum CMD_LIST
{
	RX_CMD_NULL,		
	RX_CMD_GET_ERP_TYPE, //checked
	RX_CMD_GET_HW_VERSION,//checked
	RX_CMD_GET_FW_VERSION,//checked
	RX_CMD_SET_MOTOR,//checked
	RX_CMD_SET_RGB,//checked
	RX_CMD_SET_LED,//checked
	RX_CMD_SET_LED_PWM,//checked
	RX_CMD_SET_SERVO,
	RX_CMD_GET_TOUCH, //checked
	RX_CMD_GET_IR,
	RX_CMD_GET_COLOUR,
	RX_CMD_GET_COLOUR_RED,
	RX_CMD_GET_COLOUR_GREEN,
	RX_CMD_GET_COLOUR_BLUE,
	RX_CMD_GET_COLOUR_CLEAR,
	RX_CMD_GET_ACC,
	RX_CMD_GET_ACC_X,
	RX_CMD_GET_ACC_Y,
	RX_CMD_GET_ACC_Z,
	RX_CMD_GET_GYRO,
	RX_CMD_GET_GYRO_X,
	RX_CMD_GET_GYRO_Y,
	RX_CMD_GET_GYRO_Z,
	RX_CMD_GET_MPU6050_TEMP,
	RX_CMD_GET_MPU6050,
	RX_CMD_GET_NRF52_TEMP,
	RX_CMD_GET_ULTRASONIC, //checked
	RX_CMD_CONFIG_PORT_SERVO180,
	RX_CMD_CONFIG_PORT_LED_PWM, //checked
	RX_CMD_CONFIG_PORT,
	RX_CMD_CONFIG_LINE_IR_THRESHOLD,
	RX_CMD_CONFIG_OBSTACLE_IR_THRESHOLD,
	RX_CMD_CALIBRATE_THRESHOLD,
	RX_CMD_START_IR_ENGINE,
	RX_CMD_ACTIVE_PERIPHERAL_INIT
}cmd_t;

class EnginoRobotics 
{
private:
	void sendCMD(cmd_t spi_cmd);
	void getBufferSPI(uint8_t * dataBuf, uint8_t len);
	uint8_t getByteSPI();
	void sendBuff(uint8_t * packet, uint8_t len);

public:
  	void Begin();
	char * getERPType();
	char * getHWVersion();
	char * getFWVersion();
    void setMotor(uint8_t port, uint8_t direction, uint8_t speed);
    void setRGB(uint8_t red, uint8_t green, uint8_t blue);  
	void setLed(uint8_t port, uint8_t state);
	void setLedPWM(uint8_t channel, uint8_t duty);
	void setServo(uint8_t channel, uint8_t angle);
    bool getTouch(uint8_t port);  
	bool getIR(uint8_t port);
	void EnginoRobotics::getColour(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
	uint16_t getColourRed();
	uint16_t getColourGreen();
	uint16_t getColourBlue();
	uint16_t getColourClear();
	void getAcc(int16_t* x, int16_t* y, int16_t* z);
	int16_t getAccX();
	int16_t getAccY();
	int16_t getAccZ();
	void getGyro(int16_t* x, int16_t* y, int16_t* z);
	int16_t getGyroX();
	int16_t getGyroY();
	int16_t getGyroZ();
	int16_t getMPU6050Temp();
	void getMPU6050(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* temp);
	void getNRF52Temp();
	uint16_t getUltrasonic();
	void configPortServo(uint8_t portA,uint8_t portB,uint8_t portC, uint8_t portD);
	void configPortLedPWM(uint8_t portA,uint8_t portB,uint8_t portC, uint8_t portD);
	void configPort(uint8_t port,uint8_t element,uint8_t state);
	void condigLineIRThreshold();
	void condigObstacleIRThreshold();
	bool calibrateIRThreshold(uint8_t port);
	void StartIREnigine();
	void ActivePeripheralInit();
};

#endif