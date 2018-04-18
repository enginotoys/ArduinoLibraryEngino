#include <enginoRobotics.h>

EnginoRobotics ERP;

//setting some randon initial values
uint8_t r = 55;
uint8_t g = 123;
uint8_t b = 0;
  
void setup(){                  
  ERP.Begin();
 
  delay(500);
}

void loop(){  
  /* ERP.setRGB(red, green, blue)
               ||     ||    ||    
               \/     \/    \/    
              RED   Green  Blue 
             Value  Value  Value
  */
  ERP.setRGB(r,g,b); // ON BOARD RGB LED Color

  //randomnly changing rgb values between 0-255
  r++;
  g--;
  b++;

  //adding some delay to slow down the effect and make it visible
  delay(10);  
}
