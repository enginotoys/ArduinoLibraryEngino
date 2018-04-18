#include <enginoRobotics.h>

EnginoRobotics ERP;

void setup(){									              //PORT NAME
	uint8_t port_config[14] = {MOTOR,	0,	  	//MOTORA  - Port which Motor is connected
            							   MOTOR,	0,		  //MOTORB
            							   EMPTY,	0,		  //MOTORC
            							   EMPTY,	0,		  //LED1
            							   EMPTY,	0,		  //LED2
            							   EMPTY,	0,		  //SENSOR1
            							   EMPTY,	0};		  //SENSOR2
	Serial.begin(115200);
	ERP.Begin();
	ERP.config_all(port_config);

	delay(500);
}

void loop(){
  // Motor Function --> ERP.setMotor(PORT_NAME, DIRECTION,  SPEED);
                   //                      ||       ||         ||   
                   //                      \/       \/         \/     
                   //                  MOTORA    CLOCKWISE     0-100  
                   //                  MOTORB  ANTICLOCKWISE
                   //                  MOTORC      OFF
                   //                             BRAKE
	
	ERP.setMotor(MOTORB,CLOCKWISE,100);  
  delay(1000);
  ERP.setMotor(MOTORB,OFF,0);
  delay(1000);
  ERP.setMotor(MOTORB,ANTICLOCKWISE,50);
  delay(1000);
  ERP.setMotor(MOTORB,BRAKE,0);
  delay(1000);
}
