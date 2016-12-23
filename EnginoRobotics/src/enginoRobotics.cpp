#include "enginoRobotics.h"
#include "SPI.h"

void EnginoRobotics::Begin()
{
	SPI.begin();
  	pinMode(CS, OUTPUT);
  	digitalWrite(CS, HIGH);

  	delay(1); //give some time for the pin to go high before sending any commands
}

void EnginoRobotics::sendCMD(cmd_t spi_cmd)
{
	SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
	digitalWrite(CS, LOW);

	SPI.transfer(spi_cmd);

	digitalWrite(CS, HIGH);
	SPI.endTransaction();

	delayMicroseconds(SPI_DELAY);
}

void EnginoRobotics::sendBuff(uint8_t * packet, uint8_t len)
{
	SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
	digitalWrite(CS, LOW);

	SPI.transfer(packet, len);

	digitalWrite(CS, HIGH);
	SPI.endTransaction();

	delayMicroseconds(SPI_DELAY);
}

uint8_t EnginoRobotics::getByteSPI()
{
	uint8_t temp;

	SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
	digitalWrite(CS, LOW);

	temp = SPI.transfer(0xFF);

	digitalWrite(CS, HIGH);
	SPI.endTransaction();

	delayMicroseconds(SPI_DELAY);

	return temp;
}

void EnginoRobotics::getBufferSPI(uint8_t * dataBuf, uint8_t len)
{
	SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
	digitalWrite(CS, LOW);

	SPI.transfer(dataBuf, len);

	digitalWrite(CS, HIGH);
	SPI.endTransaction();

	delayMicroseconds(SPI_DELAY);
}

char * EnginoRobotics::getERPType()
{
	uint8_t buffer[10] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

	sendCMD(RX_CMD_GET_ERP_TYPE);

	getBufferSPI(buffer, 10);

	return buffer;
} 

char * EnginoRobotics::getHWVersion()
{
	uint8_t buffer[7] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

	sendCMD(RX_CMD_GET_HW_VERSION);

	getBufferSPI(buffer, 7);

	return buffer;
} 

char * EnginoRobotics::getFWVersion()
{
	uint8_t buffer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

	sendCMD(RX_CMD_GET_FW_VERSION);

	getBufferSPI(buffer, 6);

	return buffer;
} 

void EnginoRobotics::setMotor(uint8_t port, uint8_t direction, uint8_t speed)
{
    uint8_t buffer[4] = {RX_CMD_SET_MOTOR, port, direction, speed};

   	sendBuff(buffer, 4);
}

void EnginoRobotics::setRGB(uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t buffer[4] = {RX_CMD_SET_RGB, red, green, blue};

   	sendBuff(buffer, 4);
}

void EnginoRobotics::setLed(uint8_t port, uint8_t state)
{
	uint8_t buffer[3] = {RX_CMD_SET_LED, port, state};

   	sendBuff(buffer, 3);
}

void EnginoRobotics::setLedPWM(uint8_t channel, uint8_t duty)
{
	uint8_t buffer[3] = {RX_CMD_SET_LED_PWM, channel, duty};

   	sendBuff(buffer, 3);
}

void EnginoRobotics::setServo(uint8_t channel, uint8_t angle)
{
	uint8_t buffer[3] = {RX_CMD_SET_SERVO, channel, angle};

   	sendBuff(buffer, 3);
}

bool EnginoRobotics::getTouch(uint8_t port)
{
	uint8_t buffer_tx[2] = {RX_CMD_GET_TOUCH, port};
	uint8_t buffer_rx[2] = {0xFF,0xFF};

   	sendBuff(buffer_tx, 2);

   	//return getByteSPI();

   	while(buffer_rx[0] != 0x55)
   	{	
   		getBufferSPI(buffer_rx, 2);
   	}
   	
   	return buffer_rx[1];
}

bool EnginoRobotics::getIR(uint8_t port)
{
	uint8_t buffer[2] = {RX_CMD_GET_IR, port};

   	sendBuff(buffer, 2);

   	return getByteSPI();
}

void EnginoRobotics::getColour(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
	uint8_t buffer[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

	sendCMD(RX_CMD_GET_COLOUR);

	getBufferSPI(buffer, 8);

 	*r = ((buffer[1] << 8) | (buffer[0]));
  	*g = ((buffer[3] << 8) | (buffer[2]));
  	*b = ((buffer[5] << 8) | (buffer[4]));
 	*c = ((buffer[7] << 8) | (buffer[6]));
} 

uint16_t EnginoRobotics::getColourRed()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_COLOUR_RED);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
} 

uint16_t EnginoRobotics::getColourGreen()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_COLOUR_GREEN);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
} 

uint16_t EnginoRobotics::getColourBlue()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_COLOUR_BLUE);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
} 

uint16_t EnginoRobotics::getColourClear()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_COLOUR_CLEAR);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
} 

void EnginoRobotics::getAcc(int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t buffer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

	sendCMD(RX_CMD_GET_ACC);

	getBufferSPI(buffer, 6);

 	*x = ((buffer[1] << 8) | (buffer[0]));
  	*y = ((buffer[3] << 8) | (buffer[2]));
  	*z = ((buffer[5] << 8) | (buffer[4]));
} 

int16_t EnginoRobotics::getAccX()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_ACC_X);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
} 

int16_t EnginoRobotics::getAccY()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_ACC_Y);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
} 

int16_t EnginoRobotics::getAccZ()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_ACC_Z);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
} 

void EnginoRobotics::getGyro(int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t buffer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

	sendCMD(RX_CMD_GET_GYRO);

	getBufferSPI(buffer, 6);

 	*x = ((buffer[1] << 8) | (buffer[0]));
  	*y = ((buffer[3] << 8) | (buffer[2]));
  	*z = ((buffer[5] << 8) | (buffer[4]));
} 

int16_t EnginoRobotics::getGyroX()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_GYRO_X);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
} 

int16_t EnginoRobotics::getGyroY()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_GYRO_Y);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
} 

int16_t EnginoRobotics::getGyroZ()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_GYRO_Z);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
} 

int16_t EnginoRobotics::getMPU6050Temp()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_MPU6050_TEMP);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
} 

void EnginoRobotics::getMPU6050(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* temp)
{
	uint8_t buffer[14] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

	sendCMD(RX_CMD_GET_MPU6050);

	getBufferSPI(buffer, 14);

 	*ax = ((buffer[1] << 8) | (buffer[0]));
  	*ay = ((buffer[3] << 8) | (buffer[2]));
  	*az = ((buffer[5] << 8) | (buffer[4]));
  	*gx = ((buffer[7] << 8) | (buffer[6]));
  	*gy = ((buffer[9] << 8) | (buffer[8]));
  	*gz = ((buffer[11] << 8) | (buffer[10]));
  	*temp = ((buffer[13] << 8) | (buffer[12]));
} 

void EnginoRobotics::getNRF52Temp()
{

} 

uint16_t EnginoRobotics::getUltrasonic()
{
	uint8_t buffer[2] = {0xFF,0xFF};

	sendCMD(RX_CMD_GET_ULTRASONIC);

	getBufferSPI(buffer, 2);

	return ((buffer[1] << 8) | (buffer[0]));
}

void EnginoRobotics::configPortServo(uint8_t portA,uint8_t portB,uint8_t portC, uint8_t portD)
{
	uint8_t buffer[5] = {RX_CMD_CONFIG_PORT_SERVO180, portA, portB, portC, portD};

   	sendBuff(buffer, 5);
}

void EnginoRobotics::configPortLedPWM(uint8_t portA,uint8_t portB,uint8_t portC, uint8_t portD)
{
	//need to give a bit more flexibility here.. allow one port setting at a time etc
	uint8_t buffer[5] = {RX_CMD_CONFIG_PORT_LED_PWM, portA, portB, portC, portD};

   	sendBuff(buffer, 5);
}

void EnginoRobotics::configPort(uint8_t port, uint8_t element,uint8_t state)
{
	uint8_t buffer[4] = {RX_CMD_CONFIG_PORT, port, element, state};

   	sendBuff(buffer, 4);
}

void EnginoRobotics::condigLineIRThreshold(uint8_t ir_th)
{
	uint8_t buffer[2] = {RX_CMD_CONFIG_LINE_IR_THRESHOLD, ir_th};

   	sendBuff(buffer, 2);
}

void EnginoRobotics::condigObstacleIRThreshold(uint8_t ir_th)
{
	uint8_t buffer[2] = {RX_CMD_CONFIG_OBSTACLE_IR_THRESHOLD, ir_th};

   	sendBuff(buffer, 2);
}

bool EnginoRobotics::calibrateIRThreshold(uint8_t port)
{
	uint8_t buffer_tx[2] = {RX_CMD_CALIBRATE_THRESHOLD, port};
	uint8_t buffer_rx[2] = {0xFF,0xFF};

   	sendBuff(buffer_tx, 2);

   	while(buffer_rx[0] != 0x55)
   	{	
   		getBufferSPI(buffer_rx, 2);
   		delay(237);
   	}
   	
   	return buffer_rx[1];
}

void EnginoRobotics::StartIREnigine()
{
	sendCMD(RX_CMD_START_IR_ENGINE);
}
