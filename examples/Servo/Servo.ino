#include <enginoRobotics.h>

EnginoRobotics ERP;

void setup() {                            //PORT NAME
  uint8_t port_config[14] = {EMPTY, 0,    //MOTORA
                             EMPTY, 0,    //MOTORB
                             EMPTY, 0,    //MOTORC
                             SERVO, 90,    //LED1  - Port which Servo Module is connected, initial position
                             EMPTY, 0,    //LED2
                             EMPTY, 0,    //SENSOR1, THRESHOLD
                             EMPTY, 0     //SENSOR2, THRESHOLD
                            };  
  Serial.begin(115200);
  ERP.Begin();
  ERP.config_all(port_config);

  delay(500);
}

void loop() {
  
  ERP.setServo180(LED1,45); //Servo is set to 45 Degrees
  delay(1000);
  ERP.setServo180(LED1,90);
  delay(1000);
/*ERP.setServo180(port, angle)
               ||     ||    
               \/     \/    
              Port   Angle  
*/
                 
}
