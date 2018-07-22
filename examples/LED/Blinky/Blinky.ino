#include <enginoRobotics.h>

EnginoRobotics ERP;

void setup(){                             //PORT NAME
  uint8_t port_config[14] = {EMPTY, 0,    //MOTORA
                             EMPTY, 0,    //MOTORB
                             EMPTY, 0,    //MOTORC
                             LED, 0,      //LED1  - Port which LED Module is connected
                             EMPTY, 0,    //LED2
                             EMPTY, 0,    //SENSOR1
                             EMPTY, 0};   //Sensor2
  ERP.Begin();
  ERP.config_all(port_config);

  delay(500);
}

void loop(){
  ERP.setLed(LED1,true);    //Turn LED at Port LED1 ON 
  delay(1000);              //For One Second
  ERP.setLed(LED1,false);   //Turn LED at Port LED1 OFF
  delay(1000);              //For One Second
}


