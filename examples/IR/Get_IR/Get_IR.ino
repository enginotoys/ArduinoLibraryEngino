#include <enginoRobotics.h>

EnginoRobotics ERP;

void setup() {                                     //PORT NAME
  uint8_t port_config[14] = {EMPTY, 0,             //MOTORA
                             EMPTY, 0,             //MOTORB
                             EMPTY, 0,             //MOTORC
                             LED, 0,               //LED1 - Port which LED Module is connected
                             EMPTY, 0,             //LED2
                             IR_OBSTACLE, 100,     //SENSOR1, THRESHOLD - Port which IR Module is connected. Threshold is the obstacle detection distance ex.100 -> biggest 
                             EMPTY, 0              //SENSOR2, THRESHOLD
                            };  
  ERP.Begin();
  ERP.config_all(port_config);

  delay(500);
}

void loop() {
  if (ERP.getIR(SENSOR1)){              //If IR is true (detect an obstacle less than 100) then  
    ERP.setLed(LED1, true, 0, 0);       //turn the LED which is Connected at port LED1 ON
  }else{                                //else
    ERP.setLed(LED1, false, 0, 0);      //leave the LED which is Connected at port LED1 OFF
  }
}
