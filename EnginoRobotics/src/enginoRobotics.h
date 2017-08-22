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

typedef enum screenBtnsEnumenum
{
	UP		= 0,
	CANCEL,
	DOWN,
	LEFT,
	OK,
	RIGHT
} screenBtn_t;

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
    RX_CMD_NULL							= 0,
    RX_CMD_GET_ERP_TYPE					= 1, //checked
    RX_CMD_GET_HW_VERSION				= 2, //checked
    RX_CMD_GET_FW_VERSION				= 3, //checked
    RX_CMD_SET_MOTOR					= 4, //checked
    RX_CMD_SET_RGB						= 5, //checked
    RX_CMD_SET_LED						= 6, //checked
    RX_CMD_SET_LED_PWM					= 7, //checked
    RX_CMD_SET_SERVO180					= 8,
    RX_CMD_GET_TOUCH					= 9, //checked
    RX_CMD_GET_IR						= 10, //checked
    RX_CMD_GET_COLOUR					= 11, //checked
    RX_CMD_GET_COLOUR_RED				= 12,
    RX_CMD_GET_COLOUR_GREEN				= 13,
    RX_CMD_GET_COLOUR_BLUE				= 14,
    RX_CMD_GET_COLOUR_CLEAR				= 15,
    RX_CMD_GET_ACC						= 16,
    RX_CMD_GET_ACC_X					= 17,
    RX_CMD_GET_ACC_Y					= 18,
    RX_CMD_GET_ACC_Z					= 19,
    RX_CMD_GET_GYRO						= 20,
    RX_CMD_GET_GYRO_X					= 21,
    RX_CMD_GET_GYRO_Y					= 22,
    RX_CMD_GET_GYRO_Z					= 23,
    RX_CMD_GET_MPU6050_TEMP				= 24,
    RX_CMD_GET_MPU6050					= 25,
    RX_CMD_GET_NRF52_TEMP				= 26,
    RX_CMD_GET_ULTRASONIC				= 27, //checked
    RX_CMD_CONFIG_PORT_SERVO180			= 28,
    RX_CMD_CONFIG_PORT_LED_PWM			= 29, //checked
    RX_CMD_CONFIG_PORT					= 30,//checked //30
    RX_CMD_CONFIG_LINE_IR_THRESHOLD		= 31,//checked
    RX_CMD_CONFIG_OBSTACLE_IR_THRESHOLD	= 32,//checked
    RX_CMD_CALIBRATE_THRESHOLD			= 33, //checked
    RX_CMD_START_IR_ENGINE 				= 34, //checked
    RX_CMD_SKETCH_UPLOAD				= 35,
    RX_CMD_SEND_MANUAL_PROGRAM			= 36,
    RX_CMD_CONFIG_ALL					= 37,
    RX_CMD_BLUETOOTH_STATE				= 38,
    RX_CMD_WIFI_STATE					= 39,
    RX_CMD_START_PLAYING				= 40,
    RX_CMD_START_LOOPING				= 41,
    RX_CMD_STOP_PLAYING					= 42,
    RX_CMD_MANUAL_PROG_MODE       		= 43,
    RX_CMD_GET_MOVE               		= 44,
    RX_CMD_SAVE_MANUAL_PROG       		= 45,
    RX_CMD_ABORT_MANUAL_PROG_MODE 		= 46,
    RX_CMD_GET_SETTINGS					= 47,
    RX_CMD_BUZZER						= 48,
    RX_CMD_ANYTHING_RUNNING				= 49,
    RX_CMD_LED_ADVANCED                 = 50,
    RX_CMD_IS_LED_RUNNING				= 51,
    RX_CMD_IS_MOTOR_RUNNING				= 52,
    RX_CMD_IS_BUZZER_RUNNING			= 53,
    RX_CMD_IS_RGB_RUNNING				= 54,
	RX_CMD_IS_SERVO_RUNNING				= 55,
	RX_CMD_SET_SERVO360					= 56,
	RX_CMD_GET_GYRO_YPR					= 62,
	RX_CMD_GET_RGB						= 63,
	RX_CMD_SET_XA_OFFSET				= 64,
	RX_CMD_SET_YA_OFFSET				= 65,
	RX_CMD_SET_ZA_OFFSET				= 66,
	RX_CMD_SET_XG_OFFSET				= 67,
	RX_CMD_SET_YG_OFFSET				= 68,
	RX_CMD_SET_ZG_OFFSET				= 69,
	RX_CMD_SET_MPU_CALIBRATING			= 70,
	RX_CMD_STOP_PROGRAM_ON_SCREEN		= 71,
	RX_CMD_SPLASH_TEXT					= 72,
	RX_CMD_GET_SCREEN_BUTTONS			= 73,
	RX_CMD_SEND_SCREEN_BUTTONS			= 74,
	RX_CMD_SET_MAG_CALIBRATING			= 75,
	MAX_CMDS
}cmd_t;

class EnginoRobotics
{
private:
	void CS_LOW();
	void CS_HIGH();
    void sendCMD(cmd_t spi_cmd);
    void getBufferSPI(uint8_t * dataBuf, uint8_t len);
    uint8_t getByteSPI();
    void sendBuff(uint8_t * packet, uint8_t len);
    
public:
    void Begin();
    bool isReady();
    uint8_t * getERPType();
    uint8_t * getHWVersion();
    uint8_t * getFWVersion();
    void setMotor(uint8_t port, uint8_t direction, uint8_t speed, uint32_t delay, uint32_t duration);
    void setBuzzer(uint16_t frequency, uint16_t delay, uint16_t duration);
    void setRGB(uint8_t red, uint8_t green, uint8_t blue, uint16_t delay, uint16_t duration);
    void setLed(uint8_t port, uint8_t state);
    void setLed(uint8_t port, uint8_t state, uint16_t delay, uint16_t duration);
    void setLedPWM(uint8_t channel, uint8_t duty);
    void setServo180(uint8_t channel, uint8_t angle, uint16_t delay, uint16_t duration);
    void setServo360(uint8_t channel, uint8_t direction, uint8_t speed, uint16_t delay, uint16_t duration);
    bool getTouch(uint8_t port);
    bool getIR(uint8_t port);
    void getColour(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
    void getRGB(uint8_t *r, uint8_t *g, uint8_t *b);
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
    void getGyroYPR(int16_t* yaw, int16_t* pitch, int16_t* roll);	
    void getNRF52Temp();
    uint16_t getUltrasonic();
    void configPortServo(uint8_t portA,uint8_t portB,uint8_t portC, uint8_t portD);
    void configPortLedPWM(uint8_t portA,uint8_t portB,uint8_t portC, uint8_t portD);
    void configPort(uint8_t port,uint8_t element,uint8_t state);
    void condigLineIRThreshold(uint8_t ir_th);
    void condigObstacleIRThreshold(uint8_t ir_th);
    uint8_t calibrateIRThreshold(uint8_t port);
    void StartIREnigine();
    bool isAnythingRunning();
    bool isLedRunning(uint8_t port);
    bool isMotorRunning(uint8_t port);
    bool isBuzzerRunning(void);
    bool isRGBRunning(void);
	bool isServoRunning(uint8_t port);
	void setXAccelOffset(uint16_t offset);
	void setYAccelOffset(uint16_t offset);
	void setZAccelOffset(uint16_t offset);
	void setXGyroOffset(uint16_t offset);
	void setYGyroOffset(uint16_t offset);
	void setZGyroOffset(uint16_t offset);
	void setMPUcalibrating(bool en);
	void stopPlaying(void);
	void print(uint16_t timeout, char s[]);
	uint8_t getScreenBtn(screenBtn_t btn);
	uint8_t calibrateMag(void);
};

#endif
