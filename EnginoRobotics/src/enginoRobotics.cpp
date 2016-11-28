#include "enginoRobotics.h"

void EnginoRobotics::motor(uint8_t port, uint8_t direction, uint8_t speed)
{
    Serial.write(0xFF);
	Serial.write(0xFA);
	Serial.write(0x08);
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