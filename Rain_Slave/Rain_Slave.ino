/*****************************************************************************
Master_Test.ino
Written By:  Bobby Schulz (schu3119@umn.edu)
Development Environment: PlatformIO 3.5.3
Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
Software License: BSD-3.
  Copyright (c) 2017, Stroud Water Research Center (SWRC)
  and the EnviroDIY Development Team

Based on Wire Reader Master by Nicholas Zambetti <http://www.zambetti.com>

This sketch is designed to be loaded onto the master device in the system and reads from the slave tipping bucket counter which is loaded with the Salve_TippingBucket_Test.ino
firmware. This program simply reads the number of tips every 2 seconds, then prints this value to the serial monitor.

The Master should request the device's rain every X seconds

Rain/X = Intensity of desired time frame

Connections to the salve device should be:
SCL -> Slave device SCL
SDA -> Slave device SDA

DISCLAIMER:
THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
*****************************************************************************/

//#include <Wire.h>  //Include wire library for I2C communication 
#include <avr/wdt.h> //Include library to turn off WDT
#include <avr/power.h> //Includ power library to shut off internal systems (REOMVE??)
#include <avr/sleep.h>  //Include sleep library to turn off logger
#include <Wire.h>


volatile unsigned long Counter; //Used to count dummy cycles
long double rain_depth = 0;
unsigned long UpdateRate = 4000; //Number of ms between serial prints, 4 seconds by default
uint8_t ADR = 0x08; //Address of slave device, 0x08 by default
volatile long NumTips = 0; //Tip counter used by ISR
long ReadTips = 0; //Used as output for sample of tip counter to store "old" value, while counter can increment, used to make code reentrant 
int Pin = 3; //Default pin value
int Pin_Low = 4;  //Just used to drive one side of the tipping bucket reed switch low, a digitial pin is only used to make wiring easier 
int Debounce = 10;  //The minimum length of pulse which will be counted be the device, all shorter pulses are assumed to be noise

int start_time = 0; //start time of intensity measurement
int current_time = 0; //time during intensity measurement
int Time_passed  = 0;// One second has passed... //Ichibio Keika
int Intensity_Period = 30000; //30 (30000 ms) second interval for measuring intensity
double Intensity_value = 0;
void setup() {
  Wire.begin(ADR);          // join i2c bus as slave with address #8
  Wire.onRequest(SendTips); //call SendTips which address is recieved
  Serial.begin(115200);  // start serial for output
  Serial.print("Oh? You're approaching me? Instead of running away, you're coming right to me?"); //Generic begin statment for monitor
  pinMode(Pin, INPUT_PULLUP); //Setup pin for tipping bucket using internal pullup 
  pinMode(Pin_Low, OUTPUT);
  digitalWrite(Pin_Low, LOW); //Drive pin adjacent to interrupt pin low, acts as "ground" for tipping bucket
  attachInterrupt(digitalPinToInterrupt(Pin), Tip, CHANGE); //Setup an interrupt for the tipping bucket pin, with Tip as the ISR, which will activate on every edge
}

void loop() {
  unsigned int tips = 0; //Used to measure the number of tips
  //delay(UpdateRate); //Waits for next period
  //Serial.print("Tips thus far are: ");
  //Serial.println(NumTips);
  double long rain = NumTips * .01;
  Intensity();
}
void Tip() {  //ISR for tipping events
  static long StartPulse = 0; //Used as variable to measure time between interrupt edges
  if(((millis() - StartPulse) > Debounce) && digitalRead(Pin)) { //Check if the last edge was more than 1 debounce time period ago, and that the edge measured is rising
    NumTips++; //If true, increment the tip counter
  }
  StartPulse = millis(); //Keep a record of the last edge time
}

bool Update() {  //Call to update the number of tips from the ISR counter variable
  ReadTips = NumTips; //Get number of tips
  NumTips = 0;  //Clear the tip counter
  return true; //Return true as a convention for EnviroDIY
}
void SendTips() {  //ISR for I2C requests
  Counter = 0;  //Clear counter, this should have happened due to the wake up, but in case a weird sequence of events occours where that is not the case, clear it again to be safe
  Update(); //Update tip counts
  //Intensity();
  Wire.write(ReadTips); //Respond with number of tips since last call
}
// next function to find intensity
void Intensity(){
if (current_time == 0){
  current_time = millis();
  start_time = current_time;
}
else if (Time_passed < Intensity_Period){
  current_time = millis();
  Time_passed = current_time-start_time;
  Intensity_value = (double)NumTips/Time_passed;
}
else {
 Intensity_value = (double)NumTips/Time_passed;
 current_time = 0;
 Serial.print("Wow we made it to the end of the measurement");
 Serial.println(Intensity_value);
}
  
}


/*****************************************************************************
Salve_TippingBucket_Test.ino
Written By:  Bobby Schulz (schu3119@umn.edu)
Development Environment: PlatformIO 3.5.3
Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
Software License: BSD-3.
  Copyright (c) 2017, Stroud Water Research Center (SWRC)
  and the EnviroDIY Development Team
This sketch is firmware to be loaded onto a slave micro-controller and used for counting pulse events, specifically those from a tipping bucket rain gauge
This sketch turns the device into a tip counter which is also an I2C slave, every time a tip occours, the device counts it, then when the master device reads from the slave
device, it returns the number of tips since the last time the salve was read in the form of an unsigned int
an example master code can be found at https://github.com/EnviroDIY/TippingBucketRainGauge under the name "Master_Test.ino"
Required connections are:
Pin (3 by default) -> Tipping bucket line A
Pin_Low (4 by default) -> Tipping bucket line B
SCL -> Master device SCL
SDA -> Master device SDA
Power -> External power (varies by device)
See readme located at https://github.com/EnviroDIY/TippingBucketRainGauge for more detailed wiring documentation and example using the Adafruit Trinket Pro
DISCLAIMER:
THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
*****************************************************************************/
/*
#include <Wire.h>  //Include wire library for I2C communication 
#include <avr/wdt.h> //Include library to turn off WDT
#include <avr/power.h> //Includ power library to shut off internal systems (REOMVE??)
#include <avr/sleep.h>  //Include sleep library to turn off logger

volatile long NumTips = 0; //Tip counter used by ISR
long ReadTips = 0; //Used as output for sample of tip counter to store "old" value, while counter can increment, used to make code reentrant 
int Pin = 3; //Default pin value
int Pin_Low = 4;  //Just used to drive one side of the tipping bucket reed switch low, a digitial pin is only used to make wiring easier 
int Debounce = 10;  //The minimum length of pulse which will be counted be the device, all shorter pulses are assumed to be noise
//In experimentation, tipping bucket pulses are close to 100ms, so a debounce period of 10ms is sufficiently conservative to never miss a pulse, but was also found to remove all 
//"bounce" errors in testing. If erronious tips seem to be counted (more tips in a period than you observe), you can try increasing this value, but caution is advised, when in doubt, scope the line
//and/or consult your friendly neighborhood electrical engineer 

//int ADR = 8; //The desired address of the slave device, can be modified if nessicary to avoid address colision
const unsigned long WAIT_TIME = 500; //The number of dummy cycles which are run when the device wakes up, each cycle is 1ms
volatile unsigned long Counter; //Used to count dummy cycles

void setup() {
  Wire.begin(ADR);                // join i2c bus as slave with address #8
  Wire.onRequest(SendTips); //call SendTips which address is recieved
  pinMode(Pin, INPUT_PULLUP); //Setup pin for tipping bucket using internal pullup 
  pinMode(Pin_Low, OUTPUT);
  digitalWrite(Pin_Low, LOW); //Drive pin adjacent to interrupt pin low, acts as "ground" for tipping bucket
  attachInterrupt(digitalPinToInterrupt(Pin), Tip, CHANGE); //Setup an interrupt for the tipping bucket pin, with Tip as the ISR, which will activate on every edge
}

void loop() {
   //Dummy loops are run in order to ensure the device does not go back to sleep before the I2C transaction is resolved 
   if(++Counter >= WAIT_TIME) {  //If we are done with dummy counts
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);  //Set deepest sleep mode
    sleep_enable(); //Enable the sleep option
    sleep_cpu (); //Waits here while in sleep mode
    //DEVICE SLEEPING HERE
    sleep_disable();  //First line after wake up
    Counter = 0;  //Clear counter
    TWCR = bit(TWEN) | bit(TWIE) | bit(TWEA) | bit(TWINT);  //Turn on I2C acknowledgement bits
    Wire.begin (ADR); //Restart I2C to ensure propper operation
    
    Test();
   }
Test();
   delay(1);  //Length of dummy loops
}


bool Update() {  //Call to update the number of tips from the ISR counter variable
  ReadTips = NumTips; //Get number of tips
  NumTips = 0;  //Clear the tip counter
  return true; //Return true as a convention for EnviroDIY
}

long GetValue() {  //Returns the stored value without updating if more than one call of the value is required per read cycle
  return ReadTips;  //Returns the already updated (old) value, not currently used 
}

void Tip() {  //ISR for tipping events
  static long StartPulse = 0; //Used as variable to measure time between interrupt edges
  if(((millis() - StartPulse) > Debounce) && digitalRead(Pin)) { //Check if the last edge was more than 1 debounce time period ago, and that the edge measured is rising
    NumTips++; //If true, increment the tip counter
  }
  StartPulse = millis(); //Keep a record of the last edge time
}

void SendTips() {  //ISR for I2C requests
  Counter = 0;  //Clear counter, this should have happened due to the wake up, but in case a weird sequence of events occours where that is not the case, clear it again to be safe
  Update(); //Update tip counts
  Wire.write(ReadTips); //Respond with number of tips since last call
}

void Test() {
  Wire.write(1); //Respond with number of tips since last call
}

void setup() 
{
  Wire.begin();
  Serial.begin(9600);
}

void loop() 
{
  byte num=0;
  
  // set the 24C256 eeprom address to 0
  Wire.beginTransmission(80);
  Wire.write(0);  // address high byte
  Wire.write(0);  // address low byte
  Wire.endTransmission();
  
  // read 1 byte, from address 0
  Wire.requestFrom(80, 1);
  while(Wire.read()) {
    num = Wire.receive();
  }
  Serial.print("num = ");
  Serial.println(num, DEC);
  
  // increment num
  num = num + 1;
  
  // write "num" to 24C256 eeprom at address zero
  Wire.beginTransmission(80);
  Wire.write(0);    // address high byte
  Wire.write(0);    // address low byte
  Wire.write(num);  // any more send starts writing
  Wire.endTransmission();
  
  // next time loop runs, it should retrieve the
  // same number it wrote last time... even if you
  // shut off the power
  delay(5000);
}
*/
