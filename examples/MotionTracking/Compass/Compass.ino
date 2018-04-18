#include <enginoRobotics.h>

EnginoRobotics ERP;

void setup() {                            //PORT NAME
  uint8_t port_config[14] = {EMPTY, 0,    //MOTORA
                             EMPTY, 0,    //MOTORB
                             EMPTY, 0,    //MOTORC
                             MPU6050,0,   //LED1 - Port which Accelerometer Module is connected
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
  int16_t gx, gy, gz;
  
  ERP.getGyroYPR(&gx, &gy, &gz);
/* ERP.getGyroYPR(yaw, pitch, roll)
                  ||     ||    || 
                  \/     \/    \/ 
                 Yaw   Pitch  Roll 
                Value  Value  Value
*/

  Serial.print("Degrees: ");
  Serial.println(gx);


  delay(1000);   
}
