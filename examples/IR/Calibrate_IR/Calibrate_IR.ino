#include <enginoRobotics.h>

EnginoRobotics ERP;

uint8_t calibration_threshold_IR_SENSOR1 = 0;

uint8_t calibrateIR(uint8_t port)
{
  ERP.setRGB(100,0,0); //set RGB led to red to indicate trying to calibrate
  Serial.println("Calibrating IR. Please place the sensor close to the object or on white background if calibrating for black line detection");
  delay(2000);
    
  calibration_threshold_IR_SENSOR1 = ERP.calibrateIRThreshold(port); //try to calibrate

  if ( (calibration_threshold_IR_SENSOR1 <= 100) && (calibration_threshold_IR_SENSOR1 >= 6) )
  {
    Serial.print("Calibration succeded! Calibration value: ");
    Serial.println(calibration_threshold_IR_SENSOR1);
    ERP.setRGB(0,100,0);
    delay(1500);
    ERP.setRGB(0,0,0);
  }
  else if (calibration_threshold_IR_SENSOR1 == 0xFE)
  {
    Serial.println("Are you sure there is an IR configured for this port?");
    Serial.println("Double check and reupload your sketch to continue.....");
    while(1)
    {
      ERP.setRGB(0,0,0);
      delay(1000);
      ERP.setRGB(100,0,0);
      delay(1000);
    }
  }
  else if (calibration_threshold_IR_SENSOR1 == 0xFF)
  {
    Serial.println("Calibration Failed. Place the sensor closer to the object or on white background if calibrating for black line detection");
    Serial.println("Reset arduino from the button on the bottom left corner of the breadboard to retry...");
    while(1)
    {
      ERP.setRGB(0,0,0);
      delay(1000);
      ERP.setRGB(100,0,0);
      delay(1000);
    }
  }
  else
  {
    Serial.println("Unknown error");
    while(1)
    {
      ERP.setRGB(100,0,100);
      delay(1000);
      ERP.setRGB(0,0,0);
      delay(1000);
    }
  }
}

void setup() {                                     //PORT NAME
  uint8_t port_config[14] = {EMPTY, 0,             //MOTORA
                             EMPTY, 0,             //MOTORB
                             EMPTY, 0,             //MOTORC
                             LED, 0,               //LED1 - Port which LED Module is connected
                             EMPTY, 0,             //LED2
                             IR_OBSTACLE, 100,     //SENSOR1, THRESHOLD - Port which IR Module is connected. Threshold is the obstacle detection distance ex.100 -> biggest 
                             EMPTY, 0              //SENSOR2, THRESHOLD
                            };  
  
  Serial.begin(115200);
  ERP.Begin();
  ERP.config_all(port_config);
  delay(500);

  calibrateIR(SENSOR1); 
  
  delay(500);
}

void loop() {
  if (ERP.getIR(SENSOR1)){              //If IR is true (detect an obstacle less than 100) then  
    ERP.setLed(LED1, true, 0, 0);       //turn the LED which is Connected at port LED1 ON
  }else{                                //else
    ERP.setLed(LED1, false, 0, 0);      //leave the LED which is Connected at port LED1 OFF
  }
}
