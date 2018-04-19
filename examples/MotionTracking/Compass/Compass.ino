#include <enginoRobotics.h>

EnginoRobotics ERP;

int16_t xOffset = 0;
int16_t yOffset = 0;

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

  //demonstrating magnetometer calibration and reading offsets before and after 
  //so that they can be reused and avoid the messy calibration
  ERP.getMagOffsets(&xOffset, &yOffset);
  Serial.print("xoffset before calibration: ");
  Serial.println(xOffset);
  Serial.print(" yoffset before calibration: ");
  Serial.println(yOffset);
  delay(100);
  
  //during this command, the magnetometer has to be rotated slowly along its axis while being level to ground
  Serial.println("Calibrating... Please rotate the magnetometer along its axis. Keep it level!");
  delay(1000);
  ERP.calibrateMag();
  delay(100);
  
  ERP.getMagOffsets(&xOffset, &yOffset);
  Serial.print("xoffset after calibration: ");
  Serial.println(xOffset);
  Serial.print(" yoffset after calibration: ");
  Serial.println(yOffset);
  delay(100);

  //not needed to send the offsets again as calibration has already done that
  //but putting this here for demonstration
  ERP.setMagOffsets(xOffset, yOffset);

  delay(100);
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


  delay(500);   
}
