#include <enginoRobotics.h>
#include <math.h>

EnginoRobotics ERP;
uint8_t gammatable[256];
uint8_t i;
uint16_t val;
uint16_t val8;

void setup()
{
  uint8_t res;
  Serial.begin(115200);
  
  ERP.configPortLedPWM(MOTORA, EMPTY, EMPTY, EMPTY);
  ERP.configPort(MOTORC, IR_OBSTACLE, NULL);
  ERP.configPort(LED1, IR_OBSTACLE, NULL);
  ERP.configPort(LED2,IR_LINE,NULL);
  ERP.configPort(SENSOR1, IR_OBSTACLE, NULL); 
  ERP.configPort(SENSOR2, ULTRASONIC, NULL); 
//  
//  ERP.calibrateIRThreshold(MOTORC);
//  ERP.calibrateIRThreshold(LED1);
//  ERP.calibrateIRThreshold(LED2);
//  ERP.calibrateIRThreshold(SENSOR1);
}

void loop()
{
  uint8_t r,g,b;
  
//  val = ERP.getColourRed()/4 ;
//  r = val & 0xFF;
//  val = ERP.getColourGreen()/4 ;
//  g = val & 0xFF; 
//  val = ERP.getColourBlue()/4 ;
//  b = val & 0xFF;
//  
//  ERP.setRGB(r,g,b);
//  
  val = ERP.getUltrasonic();
  val8 = val & 0xFF;
  ERP.setMotor(0x01,0x20,val8);
//  if((!ERP.getTouch(0x04)) && (ERP.getIR(0x02)))
//  {
//    ERP.setMotor(0x01,0x20,val8);
//  }
//  else
//  {
//    ERP.setMotor(0x01,0x30,0x00);
//  }
 
  delay(10);
  
//  for (i=0; i<15; i++)
//  {
//    ERP.setRGB(i,i,i);
//  }

  ERP.getIR(MOTORC);
  ERP.getIR(LED1);
  ERP.getIR(LED2);
  ERP.getIR(SENSOR1);
  
  for (i=0; i<10; i++)
  {
    ERP.setLedPWM(0, i);
  }
}
