#include <enginoRobotics.h>

EnginoRobotics ERP;

void setup() {                            //PORT NAME
  uint8_t port_config[14] = {EMPTY, 0,    //MOTORA
                             EMPTY, 0,    //MOTORB
                             EMPTY, 0,    //MOTORC
                             EMPTY, 0,    //LED1
                             LED, 0,      //LED2 - Port which LED Module is connected
                             EMPTY, 0,    //SENSOR1
                             TOUCH, 0     //SENSOR2 - Port which Touch Module is connected
                            };  
  ERP.Begin();
  ERP.config_all(port_config);

  delay(500);
}

void loop() {

  if (ERP.getTouch(SENSOR2)){       //If Touch is true (Pressed) then
    ERP.setLed(LED2, true);   //turn the LED which is Connected at port LED1 ON
  }else{                            //else
    ERP.setLed(LED2, false);  //leave the LED which is Connected at port LED1 OFF
  }

}
