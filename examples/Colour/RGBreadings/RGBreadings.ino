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

  //color correction matrix, no correction applied at the moment
  int16_t CCfactors[9] =  {10000, 0, 0,
                           0, 10000, 0,
                           0, 0, 10000};

  Serial.begin(115200);
  ERP.Begin();
  ERP.config_all(port_config);

  delay(500);

  ERP.setWhiteBalance(); //calibrating ambient or based on a specific white subject 
                         //such that the rest of the colour appear normal when RGB led is used to present them 
  
  ERP.setCCfactors(CCfactors); //colour correction based on the matrix defined above
  //this matrix gets multiplied by the white balanced raw readings to get the colour correct representation of what the sensor is seeing  
  
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
