#include <enginoRobotics.h>

EnginoRobotics ERP;

void setup() {                         
  ERP.Begin();

  delay(500);
}

void loop() {
  for (uint16_t i = 500; i < 1000; i++)
    ERP.setBuzzer(i,0,0);

  for (uint16_t i = 1000; i > 500; i--)
    ERP.setBuzzer(i,0,0);
}
