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

  ERP.setWhiteBalance(); //calibrating ambient or based on a specific white subject 
                         //such that the rest of the colour appear normal when RGB led is used to present them 
  delay(500);
}

void loop() {
  uint8_t red, green, blue;
  
  ERP.getRGB(&red,&green,&blue); // Color Sensor Function 
  ERP.setRGB(red,green,blue);
  
  //Print Readings to Serial Monitor
  Serial.print("Red:");
  Serial.println(red);
  Serial.print("Green:");
  Serial.println(green);
  Serial.print("Blue:");
  Serial.println(blue);
  Serial.println();
  
  delay(1000);
  
}
