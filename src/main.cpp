/*
  * Ultrasonic sensor example
  * Modified by Rchard Hosking from Dec 2024
  * Tank water level monitor
  * Using DFRobot A01NYUB ultrasonic transducer
  * RTC unit to wake up processor periodically
  * LoRaWAN board to send data to TTN
  * Solar panel and battery to power the system
*/

#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>
#include "LowPower.h"
#include "uSsensor.h"
#include "SparkFun_RV8803.h"

// Software Serial port pins for US sensor
#define Tx 10
#define Rx 11

// Distance variable
float distance;
// Baud rate for serial port
long BAUD = 9600;
// Inverse flag for serial port
bool inverse = false;

// RTC interrupt on pin 2 
const byte RTCint = 2;

// supply power to US transducer via a pin on the arduino
// transducer takes about 15mA when powered up 
const byte sensor_power = 4;

// Declare instance of US sensor
USsensor sensor(Tx,Rx, inverse);

// and RTC
RV8803 rtc;
//The below variables control what the date and time will be set to when set manually 
int sec = 2;
int minute = 40;
int hour = 10; //Set things in 24 hour mode
int date = 15;
int month = 1;
int year = 2025;
int weekday = 3;

long ticks = 3840;  // 1 minute between interrupts 15.625 mSec per tick

// Instance of LowPowerClass
// Seems to be declared implicitly

// Interrupt Service Routine (ISR) for RTC
void serviceRTC ()
{
  noInterrupts ();          // interrupts are disabled  
  detachInterrupt (digitalPinToInterrupt(RTCint)); // stop LOW interrupt on D2
  digitalWrite (RTCint, HIGH);  // 
  interrupts ();           // interrupts allowed now, next instruction WILL be executed
}  

void sleepNow ()
{
  noInterrupts ();          // interrupts are disabled
  attachInterrupt (digitalPinToInterrupt(RTCint), serviceRTC, LOW);  // wake up on low level on D2
  interrupts ();                                                   // interrupts allowed now, next instruction WILL be executed
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);             // sleep indefinitely, ADC and Brown out detector off
}  


void setup(){
  // Start serial port to board for debugging purposes
 Serial.begin(9600);
 delay(200);
 Serial.println("Serial port active");

// Set up pins - hardware interrupt on pin 2 for RTC
 pinMode(RTCint, INPUT);
 digitalWrite (RTCint, HIGH);
  
 pinMode (sensor_power, OUTPUT);
  
// "Tx" is the output from the transducer
// "Rx" sets mode - when high the transducer outputs a "processed" value 
// when low it outputs raw data
 pinMode(Tx, INPUT);
 pinMode(Rx, OUTPUT);
 // Set transducer mode to "processing"
 digitalWrite(Rx, HIGH);

 // Start Software Serial port
 sensor.begin(BAUD); 
 delay(300);
 if (sensor.ser.isListening()){
   Serial.println("Sensor listening");
 }
 else{
   Serial.println("Sensor not listening");
 }
 delay(20);

 // Start I2C port for RTC
 Wire.begin();
  delay(20);

  if (rtc.begin() == false)
  {
    Serial.println("RTC Device not found. Please check wiring. ");
    while(1);
  }
  else
  {
    Serial.println("RTC Device found");
  }
   // Uncomment this section to manually set time on RTC
   /*if (rtc.setTime(sec, minute, hour, weekday, date, month, year) == false) {
    Serial.println("Something went wrong setting the time");
    }
  rtc.set24Hour(); */

  // Set up RTC to wake up processor periodically
  rtc.disableAllInterrupts();
  rtc.clearAllInterruptFlags();//Clear all flags in case any interrupts have occurred.
  rtc.setCountdownTimerFrequency(COUNTDOWN_TIMER_FREQUENCY_64_HZ);
  rtc.setCountdownTimerClockTicks(ticks);  // 15.625 mSec per tick about 1 minute between interrupts
  rtc.enableHardwareInterrupt(TIMER_INTERRUPT);
  rtc.setCountdownTimerEnable(ALARM_ENABLE); //This will start the timer on the last clock tick of the I2C transaction
    
}
 
  
void loop()
{  
  
  if(rtc.getInterruptFlag(FLAG_TIMER)){rtc.clearInterruptFlag(FLAG_TIMER);}
  
  if (rtc.updateTime() == true) //Updates the time variables from RTC
  {
    //String currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird)
    String currentDate = rtc.stringDate(); //Get the current date in dd/mm/yyyy format
    String currentTime = rtc.stringTime(); //Get the time
    Serial.print(currentDate);
    Serial.print(" ");
    Serial.println(currentTime);
  }
  else
  {
    Serial.print("RTC read failed");
  }
  
  // Processor has been woken up by external interrupt 
  // Turn on US sensor
  digitalWrite(sensor_power, HIGH);
  // Allow time for sensor to wakeup and measure
  delay(1000);
  sensor.begin(BAUD); 
  delay(300);
  // Seem to need to clear buffer and then have a delay to work properly
  sensor.clearBuffer();
  delay(500);
  sensor.receiveFrame();
  sensor.measure();
  if (sensor.last_operate_status == STA_OK) {
    Serial.print("Distance = ");
    Serial.print(sensor.distance);
    Serial.println("mm");
  }
  else{
    Serial.print("Error code = ");
    if(sensor.last_operate_status == STA_ERR_CHECKSUM){
      Serial.println("Checksum error");
    }
    else if(sensor.last_operate_status == STA_ERR_CHECK_LOW_LIMIT){
      Serial.println("Below lower limit");
    }
    else if(sensor.last_operate_status == STA_ERR_CHECK_OUT_LIMIT){
      Serial.println("Above upper limit");
    }
    else if(sensor.last_operate_status == STA_ERR_DATA){
      Serial.println("Data error");
    }
    else if(sensor.last_operate_status == STA_ERR_SERIAL){
      Serial.println("Serial error");
    }
    else{
      Serial.println("Unknown error");
    }
  }
 
Serial.println("Going to sleep");
delay(100);  // delay to allow serial write to finish before sleep
// Turn off US sensor
digitalWrite(sensor_power, LOW);

// stop data from sensor or software serial interrupts will prevent sleep mode
sensor.clearBuffer();
sensor.ser.end();

// need if to allow for RTC line still being low 
if (digitalRead(RTCint) == HIGH){
  sleepNow();
}
delay(100);
Serial.println("Woken Up");

//delay(2100);

}




  