#include <enginoRobotics.h>

EnginoRobotics ERP;

void setup() {                                //PORT NAME
  uint8_t port_config[14] = {EMPTY, 0,        //MOTORA
                             EMPTY, 0,        //MOTORB
                             EMPTY, 0,        //MOTORC
                             ULTRASONIC, 0,   //LED1  - Port which Ultrasonic Module is connected
                             LED, 0,          //LED2  - Port which LED Module is connected
                             EMPTY, 0,        //SENSOR1, THRESHOLD
                             EMPTY, 0         //SENSOR2, THRESHOLD
                            };  
  Serial.begin(115200);
  ERP.Begin();
  ERP.config_all(port_config);

  delay(500);
}

void loop() {
  uint16_t distance_cm = ERP.getUltrasonic();
  
  if (distance_cm < 40){  //If Ultrasonic is less than 40cm then
    ERP.setLed(LED2, true);    //turn LED ON
  }else{                          //else
    ERP.setLed(LED2, false);   //keep LED OFF
  }

  Serial.println(distance_cm); // Display the Ultrasonic Reading on Serial Monitor

  delay(40); //ultrasonic will have a new value every 40ms or so
}
