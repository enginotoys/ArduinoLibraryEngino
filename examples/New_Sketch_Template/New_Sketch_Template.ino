#include <enginoRobotics.h>

EnginoRobotics ERP;

void setup(){                             //PORT NAME
  uint8_t port_config[14] = {EMPTY, 0,    //MOTORA
                             EMPTY, 0,    //MOTORB
                             EMPTY, 0,    //MOTORC
                             EMPTY, 0,    //LED1
                             EMPTY, 0,    //LED2
                             EMPTY, 0,    //SENSOR1
                             EMPTY, 0};   //Sensor2
  Serial.begin(115200);
  ERP.Begin();
  ERP.config_all(port_config);

  delay(500);
}

void loop(){

  delay(10);
}


