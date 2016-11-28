#ifndef __ENGINOROBOTICS_H__
#define __ENGINOROBOTICS_H__

#include <Arduino.h>
#include <inttypes.h>

class EnginoRobotics 
{
  public:
    void motor(uint8_t port, uint8_t direction, uint8_t speed);
    void setRGB(uint8_t red, uint8_t green, uint8_t blue);  
};

#endif

/*


	RX_CMD_SET_LED,
	RX_CMD_SET_LED_PWM,
	RX_CMD_SET_SERVO,
	RX_CMD_GET_TOUCH,
	RX_CMD_GET_IR,
	RX_CMD_CONFIG_PORT_SERVO180,
	RX_CMD_CONFIG_PORT_LED_PWM,
	RX_CMD_CONFIG_PORT,
	RX_CMD_START_IR_ENGINE,
	RX_CMD_ACTIVE_PERIPHERAL_INIT,*/
