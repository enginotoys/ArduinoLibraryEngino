#include <SPI.h>
#include <enginoRobotics.h>

EnginoRobotics ERP;

uint8_t buffer[10] = {};
uint8_t i = 0;
uint8_t j = 0;
uint8_t k = 0;

// our RGB -> eye-recognized gamma color
byte gammatable[256];
// set to false if using a common cathode LED
#define commonAnode false

void setup() 
{
  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i=0; i<256; i++) 
  {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
      
    gammatable[i] = x;      
  }
  
  Serial.begin(115200);
  ERP.Begin();
  ERP.configPortLedPWM(MOTORA, EMPTY, EMPTY, EMPTY);
  ERP.configPort(SENSOR2, ULTRASONIC, NULL);
  ERP.configPort(LED2, COLOUR, false);
  //ERP.configPort(LED1, TOUCH, NULL);

//  ERP.configPort(MOTORC, IR_OBSTACLE, NULL);
//  ERP.configPort(LED1, IR_OBSTACLE, NULL);
//  ERP.configPort(LED2,IR_LINE,NULL);
//  ERP.configPort(SENSOR1, IR_OBSTACLE, NULL); 
//  ERP.configPort(SENSOR2, ULTRASONIC, NULL); 
//
//  delay(10);
//  Serial.println("0");
//  
//  if (ERP.calibrateIRThreshold(MOTORC)) 
//    Serial.println("IR @ motorC calibrated");
//  else
//    Serial.println("IR @ motorC failed calibration, running ata default");
//  
//  if (ERP.calibrateIRThreshold(LED1)) 
//    Serial.println("IR @ led1 calibrated");
//  else
//    Serial.println("IR @ led1 failed calibration, running ata default");
//  
//  if (ERP.calibrateIRThreshold(LED2)) 
//    Serial.println("IR @ led2 calibrated");
//  else
//    Serial.println("IR @ led2 failed calibration, running ata default");
//  
//  if (ERP.calibrateIRThreshold(SENSOR1)) 
//    Serial.println("IR @ sensor1 calibrated");
//  else
//    Serial.println("IR @ sensor1 failed calibration, running ata default");
//    
  Serial.println("Engino Initialiased");
}

void loop() 
{
  uint8_t val8;
  uint16_t val;
  uint16_t clear, red, green, blue;

  ERP.getColour(&red, &green, &blue, &clear);
    
  Serial.print("C:\t"); Serial.print(clear);
  Serial.print("\tR:\t"); Serial.print(red);
  Serial.print("\tG:\t"); Serial.print(green);
  Serial.print("\tB:\t"); Serial.print(blue);

  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  Serial.print("\t");
  Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  Serial.println();
Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );
  ERP.setRGB(r,g,b);
  /*Serial.write(ERP.getERPType(),10);
  Serial.println();

  Serial.write(ERP.getHWVersion(),7);
  Serial.println();

  Serial.write(ERP.getFWVersion(),6);
  Serial.println();*/

  val = ERP.getUltrasonic();
  val8 = val & 0xFF;
  
  ERP.setMotor(MOTORB, CLOCKWISE, val8);

//  ERP.setRGB(i,j,k);
//  i++;
//  j++;
//  k++; 
  
  delay (50);
}
