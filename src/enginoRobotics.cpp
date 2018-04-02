#include "enginoRobotics.h"
#include "SPI.h"

void EnginoRobotics::Begin()
{
  SPI.begin();
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
    
  delay(1); //give some time for the pin to go high and the SPI to initialise before sending any commands
}

//Chip select low, start spi transaction function
void EnginoRobotics::CS_LOW()
{
  //sending two just to even out the time before and after sending the spi data
	digitalWrite(CS, LOW);
	digitalWrite(CS, LOW);
}

//Chip select high, end spi transaction function
void EnginoRobotics::CS_HIGH()
{
	digitalWrite(CS, HIGH);
  //the following delay ensures that the nrf52 has time to prepare it's buffers for the next transfer
	delayMicroseconds(200);
}

//sending a command to the main nRF52 MCU to ask for something
//commands are defined in CMD_LIST enumaration
void EnginoRobotics::sendCMD(cmd_t spi_cmd)
{
  //prepare the SPI bus with the correct settings
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  CS_LOW();
    
  SPI.transfer(spi_cmd);
    
  CS_HIGH();
  //freeing the spi bus for other peripherals to use inbetween our transactions
  SPI.endTransaction(); 
}

//sending an advanced command with data for the nRF52 MCU
void EnginoRobotics::sendBuff(uint8_t * packet, uint8_t len)
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  CS_LOW();
    
  SPI.transfer(packet, len);
    
  CS_HIGH();
  SPI.endTransaction();
}

//get back a single byte of reply from the nRF52 MCU
uint8_t EnginoRobotics::getByteSPI()
{
  uint8_t temp;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  CS_LOW();
    
  //each SPI transfer does both sending and receiving. To avoid sending rubbish to nRF52 and potentially triggering something, 
  //we send 0xFF which is reserved blank command
  temp = SPI.transfer(0xFF); 
    
  CS_HIGH();
  SPI.endTransaction();
    
  return temp;
}


//getting back an advanced reply to a command with multiple reply data
void EnginoRobotics::getBufferSPI(uint8_t * dataBuf, uint8_t len)
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  CS_LOW();
    
  SPI.transfer(dataBuf, len);
    
  CS_HIGH();
  SPI.endTransaction();
}

//command to verify we are actually talking to the correct hardware.
//correct reply is ERPPRODUINO
uint8_t * EnginoRobotics::getERPType()
{
  uint8_t buffer[10] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_ERP_TYPE);
    
  getBufferSPI(buffer, 10);
    
  return buffer;
}

//get the hardware version of the device
//REV_C.0 onwards is the production version
uint8_t * EnginoRobotics::getHWVersion()
{
  uint8_t buffer[7] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_HW_VERSION);
    
  getBufferSPI(buffer, 7);
    
  return buffer;
}

//get the firmware version of the nRF52 MCU
//need a minimum of V0.2. for this library
uint8_t * EnginoRobotics::getFWVersion()
{
  uint8_t buffer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_FW_VERSION);
    
  getBufferSPI(buffer, 6);
    
  return buffer;
}

//Function for turning the motors on ports A-C ON and OFF at a certain speed 0-100% and direction (OFF, BRAKE, CLOCKWISE, ANTICLOCKWISE)
void EnginoRobotics::setMotor(uint8_t port, uint8_t direction, uint8_t speed)
{
  if ( (port != MOTORA) || (port != MOTORB) || (port != MOTORC) )
    //port cannot drive a motor
    return;

  if ( (direction != OFF) || (direction != BRAKE) || (direction != CLOCKWISE) || (direction != ANTICLOCKWISE) )
    //impossible direction
    return;

  if (speed > 100)
    speed = 100;

  uint8_t buffer[12] = {RX_CMD_SET_MOTOR, port, direction, speed, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
     
  sendBuff(buffer, 12);
}

//Function for turning the motors on ports A-C ON and OFF at a certain speed 0-100% and direction (OFF, BRAKE, CLOCKWISE, ANTICLOCKWISE)
//for a specific amount of delay time and duration time in a non blocking fashion
void EnginoRobotics::setMotor(uint8_t port, uint8_t direction, uint8_t speed, uint32_t delay, uint32_t duration)
{
  if ( (port != MOTORA) || (port != MOTORB) || (port != MOTORC) )
    //port cannot drive a motor
    return;

  if ( (direction != OFF) || (direction != BRAKE) || (direction != CLOCKWISE) || (direction != ANTICLOCKWISE) )
    //impossible direction
    return;

  if (speed > 100)
    speed = 100;

  uint8_t buffer[12] = {RX_CMD_SET_MOTOR, port, direction, speed, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
  //assembling the buffer according to protocol
  buffer[4] = (uint8_t) ((delay & 0x000000FF) >> 0);
  buffer[5] = (uint8_t) ((delay & 0x0000FF00) >> 8);
  buffer[6] = (uint8_t) ((delay & 0x00FF0000) >> 16);
  buffer[7] = (uint8_t) ((delay & 0xFF000000) >> 24);
  buffer[8] = (uint8_t) ((duration & 0x000000FF) >> 0);
  buffer[9] = (uint8_t) ((duration & 0x0000FF00) >> 8);
  buffer[10] = (uint8_t) ((duration & 0x00FF0000) >> 16);
  buffer[11] = (uint8_t) ((duration & 0xFF000000) >> 24);
    
  sendBuff(buffer, 12);
}

//function for turning the embedded RGB led on in RGB colours or off. All variables expect a value between 0-255
void EnginoRobotics::setRGB(uint8_t red, uint8_t green, uint8_t blue)
{
  uint8_t buffer[8] = {RX_CMD_SET_RGB, red, green, blue, 0, 0, 0, 0};
    
  sendBuff(buffer, 8);
}

//function for turning the embedded RGB led on in RGB colours or off. All variables expect a value between 0-255
//for a specific amount of delay time and duration time in a non blocking fashion
void EnginoRobotics::setRGB(uint8_t red, uint8_t green, uint8_t blue, uint16_t delay, uint16_t duration)
{
  uint8_t buffer[8] = {RX_CMD_SET_RGB, red, green, blue, (delay1 >> 8), delay, (duration >> 8), duration};
    
  sendBuff(buffer, 8);
}

//function for turning the embedded buzzer on or off at a specific frequency. frequency can take any value from 500 to 15000 and 0 as the off state
//for a specific amount of delay time and duration time in a non blocking fashion
void EnginoRobotics::setBuzzer(uint16_t frequency)
{
  if (frequency < 500)
    frequency = 500;
  else if (frequency > 15000)
    frequency = 15000;

  uint8_t buffer[7] = {RX_CMD_BUZZER, (frequency >> 8), frequency,  0, 0, 0, 0};
    
  sendBuff(buffer, 7); 
}

//function for turning the embedded buzzer on or off at a specific frequency. frequency can take any value from 0 to 20000 with 0 being the off state
//for a specific amount of delay time and duration time in a non blocking fashion
void EnginoRobotics::setBuzzer(uint16_t frequency, uint16_t delay, uint16_t duration)
{
  if (frequency < 500)
    frequency = 500;
  else if (frequency > 15000)
    frequency = 15000;

  uint8_t buffer[7] = {RX_CMD_BUZZER, (frequency >> 8), frequency,  (delay >> 8), delay, (duration >> 8), duration};
    
  sendBuff(buffer, 7); 
}

//function for turning the standard LEDs that are connected on normal ports ON or OFF. state is true for on and false for off
//port can be anything: MOTORA MOTORB MOTORC LED1 LED2 SENSOR1 SENSOR2
void EnginoRobotics::setLed(uint8_t port, uint8_t state)
{
  if (port > PORT_NUMBER)
    //port is impossible
    return;

  if (state > 1)
    state = 1; //making sure we only parse true or false

  uint8_t buffer[7] = {RX_CMD_LED_ADVANCED, port, state, 0, 0, 0, 0};
    
  sendBuff(buffer, 7);
}

//function for turning the standard LEDs that are connected on normal ports ON or OFF. state is true for on and false for off
//for a specific amount of delay time and duration time in a non blocking fashion
//port can be anything: MOTORA MOTORB MOTORC LED1 LED2 SENSOR1 SENSOR2
void EnginoRobotics::setLed(uint8_t port, uint8_t state, uint16_t delay, uint16_t duration)
{
  if (port > PORT_NUMBER)
    //port is impossible
    return;

  if (state > 1)
    state = 1; //making sure we only parse true or false

  uint8_t buffer[7] = {RX_CMD_LED_ADVANCED, port, state, (delay >> 8), delay, (duration >> 8), duration};
    
  sendBuff(buffer, 7);
}

//function for operating the servo that is connected on any of the ports (MOTORA MOTORB MOTORC LED1 LED2 SENSOR1 SENSOR2)
//to a specific angle between 0-180 as fast as possible
void EnginoRobotics::setServo180(uint8_t port, uint8_t angle)
{
  if (port > PORT_NUMBER)
    //port is impossible
    return;

  if (angle > 180)
    angle = 180;

  uint8_t buffer[7] = {RX_CMD_SET_SERVO180, port, angle, 0, 0, 0, 0};
    
  sendBuff(buffer, 7);
}

//function for operating the servo that is connected on any of the ports (MOTORA MOTORB MOTORC LED1 LED2 SENSOR1 SENSOR2)
//angle is the target angle and can be anything between 0-180. setting an amount of delay in ms, would delay the start of the move
//and by setting a duration in ms, the actual transition would try to be completed in that amount of time
//for example moving from 0 to 180, if we set a duration of 5 seconds, the move would take 5 seconds; the shaft would be moving quite slow
void EnginoRobotics::setServo180(uint8_t port, uint8_t angle, uint16_t delay, uint16_t duration)
{
  if (port > PORT_NUMBER)
    //port is impossible
    return;

  if (angle > 180)
    angle = 180;

  uint8_t buffer[7] = {RX_CMD_SET_SERVO180, port, angle, (delay >> 8), delay, (duration >> 8), duration};
    
  sendBuff(buffer, 7);
}

//function for getting the state of a touch sensor that is connected on any of the ports (MOTORA MOTORB MOTORC LED1 LED2 SENSOR1 SENSOR2)
bool EnginoRobotics::getTouch(uint8_t port)
{
  if (port > PORT_NUMBER)
    //port is impossible
    return false;

  uint8_t buffer_tx[2] = {RX_CMD_GET_TOUCH, port};
    
  sendBuff(buffer_tx, 2);
    
  return getByteSPI();
}

//function for getting the state of an IR sensor that is connected on any of the ports (MOTORA MOTORB MOTORC LED1 LED2 SENSOR1 SENSOR2)
bool EnginoRobotics::getIR(uint8_t port)
{
  if (port > PORT_NUMBER)
    //port is impossible
    return false;

  uint8_t buffer_tx[2] = {RX_CMD_GET_IR, port};
    
  sendBuff(buffer_tx, 2);
   	
  return getByteSPI();
}

//this function automatically calibrates the IR at the specified port. valid ports are all ports (MOTORA MOTORB MOTORC LED1 LED2 SENSOR1 SENSOR2)
//to properly calibrate, the sensor should be facing the obstacle or be pointing at white background. (NOT black!)
//returns the actual value it managed to calibrate at (0-100) or the failure error code:
//0xFF - impossible to calibrate, object too far
//0xFE - there is no IR configured on the spedified port
uint8_t EnginoRobotics::calibrateIRThreshold(uint8_t port)
{
  if (port > PORT_NUMBER)
    //port is impossible
    return false;

  uint8_t buffer_tx[2] = {RX_CMD_CALIBRATE_THRESHOLD, port};
  uint8_t buffer_rx[2] = {0xFF,0xFF};
    
  sendBuff(buffer_tx, 2);
    
  while(buffer_rx[0] != 0x55) //nRF52 would keep replying 0xFF while busy and 0x55 once is ready
  {
    getBufferSPI(buffer_rx, 2);
    delay(237); //arbitrary number just to allow some time to calibrate and reply
  }
    
  return buffer_rx[1];
}

//function for getting the red green blue clear illuminosity levels of a colour sensor connected on one of the compatible ports (LED1 LED2 SENSOR1 SENSOR2)
//will return all 0 values if sensor is not available and configured properly
void EnginoRobotics::getColour(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  uint8_t buffer[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_COLOUR);
    
  getBufferSPI(buffer, 8);
    
  *r = ((buffer[1] << 8) | (buffer[0]));
  *g = ((buffer[3] << 8) | (buffer[2]));
  *b = ((buffer[5] << 8) | (buffer[4]));
  *c = ((buffer[7] << 8) | (buffer[6]));
}

//function for getting the red illuminosity level of a colour sensor connected on one of the compatible ports (LED1 LED2 SENSOR1 SENSOR2)
//will return 0 if sensor is not available and configured properly
uint16_t EnginoRobotics::getColourRed()
{
  uint8_t buffer[2] = {0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_COLOUR_RED);
    
  getBufferSPI(buffer, 2);
    
  return ((buffer[1] << 8) | (buffer[0]));
}

//function for getting the green illuminosity level of a colour sensor connected on one of the compatible ports (LED1 LED2 SENSOR1 SENSOR2)
//will return 0 if sensor is not available and configured properly
uint16_t EnginoRobotics::getColourGreen()
{
  uint8_t buffer[2] = {0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_COLOUR_GREEN);
    
  getBufferSPI(buffer, 2);
    
  return ((buffer[1] << 8) | (buffer[0]));
}

//function for getting the blue illuminosity level of a colour sensor connected on one of the compatible ports (LED1 LED2 SENSOR1 SENSOR2)
//will return 0 if sensor is not available and configured properly
uint16_t EnginoRobotics::getColourBlue()
{
  uint8_t buffer[2] = {0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_COLOUR_BLUE);
    
  getBufferSPI(buffer, 2);
    
  return ((buffer[1] << 8) | (buffer[0]));
}

//function for getting the clear illuminosity level (essentially brightness) of a colour sensor connected on one of the compatible ports (LED1 LED2 SENSOR1 SENSOR2)
//will return 0 if sensor is not available and configured properly
uint16_t EnginoRobotics::getColourClear()
{
  uint8_t buffer[2] = {0xFF,0xFF};
   
  sendCMD(RX_CMD_GET_COLOUR_CLEAR);
    
  getBufferSPI(buffer, 2);
    
  return ((buffer[1] << 8) | (buffer[0]));
}

//this function gets the RGB values of the colour sensor instead of the raw colour illuminosities
//it also gamma corrects the values to be correct to the human eye. values returned are from 0-255 standard RGB
//EXPERIMENTAL, colours might not be correct to the eye from this function
void EnginoRobotics::getRGB(uint8_t *r, uint8_t *g, uint8_t *b)
{
  uint8_t buffer[3] = {0xFF, 0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_RGB);
    
  getBufferSPI(buffer, 3);
  
  uint8_t r_init =  (buffer[0]);
  uint8_t g_init =  (buffer[1]);
  uint8_t b_init =  (buffer[2]);
  uint8_t highest = 0;
  if(r_init > highest){
    highest = r_init;
  }
  if(g_init > highest){
    highest = g_init;
  }
  if(b_init > highest){
    highest = b_init;
  }
  uint8_t multiplier = 255 / highest;
    *r = r_init * multiplier;
    *g = g_init * multiplier;
    *b = b_init * multiplier;
}

//function for getting the raw accelerometer data from its 3 axis.
//values range -32765 - +32765 full scale range is 4G
void EnginoRobotics::getAcc(int16_t* x, int16_t* y, int16_t* z)
{
  uint8_t buffer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_ACC);
    
  getBufferSPI(buffer, 6);
    
  *x = ((buffer[1] << 8) | (buffer[0]));
  *y = ((buffer[3] << 8) | (buffer[2]));
  *z = ((buffer[5] << 8) | (buffer[4]));
}

//function for getting the raw accelerometer data from x axis.
//values range -32765 - +32765 full scale range is 4G
int16_t EnginoRobotics::getAccX()
{
  uint8_t buffer[2] = {0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_ACC_X);
    
  getBufferSPI(buffer, 2);
    
  return ((buffer[1] << 8) | (buffer[0]));
}

//function for getting the raw accelerometer data from y axis.
//values range -32765 - +32765 full scale range is 4G
int16_t EnginoRobotics::getAccY()
{
  uint8_t buffer[2] = {0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_ACC_Y);
    
  getBufferSPI(buffer, 2);
    
  return ((buffer[1] << 8) | (buffer[0]));
}

//function for getting the raw accelerometer data from z axis.
//values range -32765 - +32765 full scale range is 4G
int16_t EnginoRobotics::getAccZ()
{
  uint8_t buffer[2] = {0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_ACC_Z);
    
  getBufferSPI(buffer, 2);
    
  return ((buffer[1] << 8) | (buffer[0]));
}

//function for getting the raw gyroscope data from its 3 axis.
//values range -32765 - +32765 full scale range is 2000 deg/sec
void EnginoRobotics::getGyro(int16_t* x, int16_t* y, int16_t* z)
{
  uint8_t buffer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_GYRO);
    
  getBufferSPI(buffer, 6);
    
  *x = ((buffer[1] << 8) | (buffer[0]));
  *y = ((buffer[3] << 8) | (buffer[2]));
  *z = ((buffer[5] << 8) | (buffer[4]));
}

//function for getting the raw gyroscope data from x axis.
//values range -32765 - +32765 full scale range is 2000 deg/sec
int16_t EnginoRobotics::getGyroX()
{
  uint8_t buffer[2] = {0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_GYRO_X);
    
  getBufferSPI(buffer, 2);
    
  return ((buffer[1] << 8) | (buffer[0]));
}

//function for getting the raw gyroscope data from x axis.
//values range -32765 - +32765 full scale range is 2000 deg/sec
int16_t EnginoRobotics::getGyroY()
{
  uint8_t buffer[2] = {0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_GYRO_Y);
    
  getBufferSPI(buffer, 2);
    
  return ((buffer[1] << 8) | (buffer[0]));
}

//function for getting the raw gyroscope data from x axis.
//values range -32765 - +32765 full scale range is 2000 deg/sec
int16_t EnginoRobotics::getGyroZ()
{
  uint8_t buffer[2] = {0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_GYRO_Z);
    
  getBufferSPI(buffer, 2);
    
  return ((buffer[1] << 8) | (buffer[0]));
}

//function for getting the raw temperature data from temperature sensor.
//EXPERIMENTAL
int16_t EnginoRobotics::getMPU6050Temp()
{
  uint8_t buffer[2] = {0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_MPU6050_TEMP);
    
  getBufferSPI(buffer, 2);
    
  return ((buffer[1] << 8) | (buffer[0]));
}

//function for getting the raw motion tracking sensor data.
void EnginoRobotics::getMPU6050(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* temp)
{
    uint8_t buffer[14] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    
    sendCMD(RX_CMD_GET_MPU6050);
    
    getBufferSPI(buffer, 14);
    
    *ax = ((buffer[1] << 8) | (buffer[0]));
    *ay = ((buffer[3] << 8) | (buffer[2]));
    *az = ((buffer[5] << 8) | (buffer[4]));
    *gx = ((buffer[7] << 8) | (buffer[6]));
    *gy = ((buffer[9] << 8) | (buffer[8]));
    *gz = ((buffer[11] << 8) | (buffer[10]));
    *temp = ((buffer[13] << 8) | (buffer[12]));
}

//this function gets the fused data from the Digital Motion Processor (DMP) of the motion tracking sensor instead of the raw axis
//using computations (quaternions) the yaw pitch and roll are returned
//value ranges: TBD
//EXPERIMENTAL: there are still improvements to be made on the readings
void EnginoRobotics::getGyroYPR(int16_t *yaw, int16_t *pitch, int16_t *roll)
{
    uint8_t buffer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    
    sendCMD(RX_CMD_GET_GYRO_YPR);
    
    getBufferSPI(buffer, 6);

    *yaw = ((buffer[1] << 8) | (buffer[0]));
    *pitch = ((buffer[3] << 8) | (buffer[2]));
    *roll = ((buffer[5] << 8) | (buffer[4]));
}

//function for getting the temperature of the nRF52 chip
//this serves as the ambient temperature reading
void EnginoRobotics::getNRF52Temp()
{
 //to be implemented in a future release   
}

//function for getting the distance in cms that the ultrasonic sees
//this can detect reliably up to 1.5m-2m
uint16_t EnginoRobotics::getUltrasonic()
{
  uint8_t buffer[3] = {0xFF,0xFF};
    
  sendCMD(RX_CMD_GET_ULTRASONIC);
    
	getBufferSPI(buffer, 2);

  return ((buffer[1] << 8) | (buffer[0]));
}


void EnginoRobotics::config_all(uint8_t * configuration)
{
  uint8_t buffer[15];

  buffer[0] = RX_CMD_CONFIG_ALL;

  for (uint8_t i = 0; i < 14; i++)
    buffer[i+1] = configuration[i];

  sendBuff(buffer, 15); 
}

//this function will force the firmware to commit suicide. Everything stops and the firmware on the arduino gets erased. Use with caution
void EnginoRobotics::stopPlaying(void)
{
  sendCMD(RX_CMD_STOP_PROGRAM_ON_SCREEN);
  
  delay(200);
  
  sendCMD(RX_CMD_STOP_PLAYING); 
  
  while(1);
}

//this functions prints a short string (up to 100 characters) on the device screen with a predifined timeout
void EnginoRobotics::print(char s[], uint16_t timeout)
{
  uint8_t buffer[100] = {RX_CMD_SPLASH_TEXT, (timeout >> 8), timeout};
  
  strcpy((char *)&buffer[3], s);

  sendBuff(buffer, (strlen(s)+3));
}

//this functions prints a short string (up to 100 characters) on the device screen
//this will only clear after a hard reset or when something else is printed on top
//EXPERIMENTAL: printing on the screen is buggy, the gui on screen is still not optimised for displaying text asynchronously to its own operations
void EnginoRobotics::print(char s[])
{
  uint8_t buffer[100] = {RX_CMD_SPLASH_TEXT, 0, 0};
  
  strcpy((char *)&buffer[3], s);

  sendBuff(buffer, (strlen(s)+3));
}

//gets the state of the on screen buttons so that they can be used in the program flow
uint8_t EnginoRobotics::getScreenBtn(screenBtn_t btn)
{
    sendCMD(RX_CMD_SEND_SCREEN_BUTTONS);
    
    return !((getByteSPI() >> btn) & 0x01);
}

uint8_t EnginoRobotics::calibrateMag(void)
{
  uint8_t buffer_rx[2] = {0xFF,0xFF};
    
    sendCMD(RX_CMD_SET_MAG_CALIBRATING);
    
    while(buffer_rx[0] != 0x55)
    {
        getBufferSPI(buffer_rx, 2);
        delay(237);
    }
    
    return buffer_rx[1];
}

//misc support functions not documented. Use with caution
bool EnginoRobotics::isAnythingRunning()
{
   	sendCMD(RX_CMD_ANYTHING_RUNNING);
    
   	return getByteSPI();
}

bool EnginoRobotics::isLedRunning(uint8_t port)
{
    uint8_t buffer_tx[2] = {RX_CMD_IS_LED_RUNNING, port};
    
   	sendBuff(buffer_tx, 2);
    
   	return getByteSPI();
}

bool EnginoRobotics::isMotorRunning(uint8_t port)
{
    uint8_t buffer_tx[2] = {RX_CMD_IS_MOTOR_RUNNING, port};
    
   	sendBuff(buffer_tx, 2);
	
   	return getByteSPI();
}

bool EnginoRobotics::isBuzzerRunning(void)
{    
   	sendCMD(RX_CMD_IS_BUZZER_RUNNING);
    
   	return getByteSPI();
}

bool EnginoRobotics::isRGBRunning(void)
{    
   	sendCMD(RX_CMD_IS_RGB_RUNNING);
    
   	return getByteSPI();
}

bool EnginoRobotics::isServoRunning(uint8_t port)
{
    uint8_t buffer_tx[2] = {RX_CMD_IS_SERVO_RUNNING, port};
    
   	sendBuff(buffer_tx, 2);
    
   	return getByteSPI();
}

void EnginoRobotics::setXAccelOffset(uint16_t offset)
{
    uint8_t buffer[3] = {RX_CMD_SET_XA_OFFSET, (offset >> 8), offset};
    
   	sendBuff(buffer, 3);
}

void EnginoRobotics::setYAccelOffset(uint16_t offset)
{
    uint8_t buffer[3] = {RX_CMD_SET_YA_OFFSET, (offset >> 8), offset};
    
   	sendBuff(buffer, 3);
}

void EnginoRobotics::setZAccelOffset(uint16_t offset)
{
    uint8_t buffer[3] = {RX_CMD_SET_ZA_OFFSET, (offset >> 8), offset};
    
   	sendBuff(buffer, 3);
}

void EnginoRobotics::setXGyroOffset(uint16_t offset)
{
    uint8_t buffer[3] = {RX_CMD_SET_XG_OFFSET, (offset >> 8), offset};
    
   	sendBuff(buffer, 3);
}

void EnginoRobotics::setYGyroOffset(uint16_t offset)
{
    uint8_t buffer[3] = {RX_CMD_SET_YG_OFFSET, (offset >> 8), offset};
    
   	sendBuff(buffer, 3);
}

void EnginoRobotics::setZGyroOffset(uint16_t offset)
{
    uint8_t buffer[3] = {RX_CMD_SET_ZG_OFFSET, (offset >> 8), offset};
    
   	sendBuff(buffer, 3);
}

void EnginoRobotics::setMPUcalibrating(bool en)
{
    uint8_t buffer[2] = {RX_CMD_SET_MPU_CALIBRATING, en};
    
   	sendBuff(buffer, 2);
}

void EnginoRobotics::testFunc(uint8_t what)
{
  uint8_t i;
  uint8_t buffer[100];
  uint8_t bufferRX[100];


  buffer[0] = RX_CMD_TEST_SPI;

  if (what == 0)
  {
    for (i = 1; i < 100; i++)
      buffer[i] = i;
  }
  else
  {
    for (i = 1; i < 100; i++)
      buffer[i] = 100 - i;
  }

  sendBuff(buffer, 100);

  for (i = 0; i < 100; i++)
      bufferRX[i] = 0;

  getBufferSPI(bufferRX, 100);
}