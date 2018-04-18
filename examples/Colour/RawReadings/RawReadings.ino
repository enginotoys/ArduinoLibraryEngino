 #include <enginoRobotics.h>

EnginoRobotics ERP;

void setup() {                            //PORT NAME
  uint8_t port_config[14] = {EMPTY, 0,    //MOTORA
                             EMPTY, 0,    //MOTORB
                             EMPTY, 0,    //MOTORC
                             COLOUR, true,   //LED1 - Port which Color Sensor is connected The property controls if illumination led should be on or off
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
  uint16_t r, g, b, c;
  
  ERP.getColour(&r,&g,&b, &c); // Color Sensor Function 
 
  //Print Readings to Serial Monitor
  Serial.print("Red: ");
  Serial.print(r);
  Serial.print("  Green: ");
  Serial.print(g);
  Serial.print("  Blue: ");
  Serial.print(b);
  Serial.print("  Clear: ");
  Serial.print(c);
  Serial.println();
  
  delay(1000);
  
}
