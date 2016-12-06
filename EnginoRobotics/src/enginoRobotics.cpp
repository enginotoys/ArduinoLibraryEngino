#include "enginoRobotics.h"

void EnginoRobotics::getERPType()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x01);
	Serial.write(0xEE);
} 

void EnginoRobotics::getHWVersion()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x02);
	Serial.write(0xEE);
} 

void EnginoRobotics::getFWVersion()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x03);
	Serial.write(0xEE);
} 

void EnginoRobotics::setMotor(uint8_t port, uint8_t direction, uint8_t speed)
{
    Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x08); //length
	Serial.write(0x04);
	Serial.write(port);
	Serial.write(direction);
	Serial.write(speed);
	Serial.write(0xEE);
}

void EnginoRobotics::setRGB(uint8_t red, uint8_t green, uint8_t blue)
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x08);
	Serial.write(0x05);
	Serial.write(red);
	Serial.write(green);
	Serial.write(blue);
	Serial.write(0xEE);
}

void EnginoRobotics::setLed(uint8_t port, uint8_t state)
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x07);
	Serial.write(0x06);
	Serial.write(port);
	Serial.write(state);
	Serial.write(0xEE);
}

void EnginoRobotics::setLedPWM(uint8_t channel, uint8_t duty)
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x07);
	Serial.write(0x07);
	Serial.write(channel);
	Serial.write(duty);
	Serial.write(0xEE);
}

void EnginoRobotics::setServo(uint8_t channel, uint8_t angle)
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x07);
	Serial.write(0x08);
	Serial.write(channel);
	Serial.write(angle);
	Serial.write(0xEE);
}

bool EnginoRobotics::getTouch(uint8_t port)
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x06);
	Serial.write(0x09);
	Serial.write(port);
	Serial.write(0xEE);
	
	while(Serial.available() == 0);
	
	return Serial.read();
}

bool EnginoRobotics::getIR(uint8_t port)
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x06);
	Serial.write(0x0A);
	Serial.write(port);
	Serial.write(0xEE);
	
	while(Serial.available() == 0);
	
	return Serial.read();
}


void EnginoRobotics::getColour()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x0B);
	Serial.write(0xEE);
} 

uint16_t EnginoRobotics::getColourRed()
{
	uint16_t val;

	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x0C);
	Serial.write(0xEE);

	while(Serial.available() != 2);

	val = Serial.read();
	val |= (Serial.read() << 8);

	return val;
} 

uint16_t EnginoRobotics::getColourGreen()
{
	uint16_t val;

	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x0D);
	Serial.write(0xEE);

	while(Serial.available() != 2);
	
	val = Serial.read();
	val |= (Serial.read() << 8);

	return val;
} 
uint16_t EnginoRobotics::getColourBlue()
{
	uint16_t val;

	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x0E);
	Serial.write(0xEE);

	while(Serial.available() != 2);
	
	val = Serial.read();
	val |= (Serial.read() << 8);

	return val;
} 

uint16_t EnginoRobotics::getColourClear()
{
	uint16_t val;

	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x0F);
	Serial.write(0xEE);

	while(Serial.available() != 2);
	
	val = Serial.read();
	val |= (Serial.read() << 8);

	return val;
} 

void EnginoRobotics::getAcc()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x10);
	Serial.write(0xEE);
} 

void EnginoRobotics::getAccX()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x11);
	Serial.write(0xEE);
} 

void EnginoRobotics::getAccY()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x12);
	Serial.write(0xEE);
} 

void EnginoRobotics::getAccZ()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x13);
	Serial.write(0xEE);
} 

void EnginoRobotics::getGyro()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x14);
	Serial.write(0xEE);
} 

void EnginoRobotics::getGyroX()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x15);
	Serial.write(0xEE);
} 


void EnginoRobotics::getGyroY()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x16);
	Serial.write(0xEE);
} 


void EnginoRobotics::getGyroZ()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x17);
	Serial.write(0xEE);
} 


void EnginoRobotics::getMPU6050Temp()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x18);
	Serial.write(0xEE);
} 


void EnginoRobotics::getMPU6050()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x19);
	Serial.write(0xEE);
} 


void EnginoRobotics::getNRF52Temp()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x1A);
	Serial.write(0xEE);
} 


uint16_t EnginoRobotics::getUltrasonic()
{
	uint16_t val;

	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x1B);
	Serial.write(0xEE);

	while(Serial.available() != 2);
	
	val = Serial.read();
	val |= (Serial.read() << 8);

	return val;
}

void EnginoRobotics::configPortServo(uint8_t portA,uint8_t portB,uint8_t portC, uint8_t portD)
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x09);
	Serial.write(0x1C);
	Serial.write(portA);
	Serial.write(portB);
	Serial.write(portC);
	Serial.write(portD);
	Serial.write(0xEE);
}

void EnginoRobotics::configPortLedPWM(uint8_t portA,uint8_t portB,uint8_t portC, uint8_t portD)
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x09);
	Serial.write(0x1D);
	Serial.write(portA);
	Serial.write(portB);
	Serial.write(portC);
	Serial.write(portD);
	Serial.write(0xEE);
}

void EnginoRobotics::configPort(uint8_t port, uint8_t element,uint8_t state){
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x08);
	Serial.write(0x1E);
	Serial.write(port);
	Serial.write(element);
	Serial.write(state);
	Serial.write(0xEE);
}

void EnginoRobotics::condigLineIRThreshold()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x1F);
	Serial.write(0xEE);
}


void EnginoRobotics::condigObstacleIRThreshold()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x20);
	Serial.write(0xEE);
}


uint8_t EnginoRobotics::calibrateIRThreshold(uint8_t port)
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x06);
	Serial.write(0x21);
	Serial.write(port);
	Serial.write(0xEE);

	while(Serial.available() == 0);
	
	return Serial.read();
}


void EnginoRobotics::StartIREnigine()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x22);
	Serial.write(0xEE);
}

void EnginoRobotics::ActivePeripheralInit()
{
	Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x05);
	Serial.write(0x23);
	Serial.write(0xEE);
}