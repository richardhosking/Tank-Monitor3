/*
  *@File  :  from DFRobot_Distance_A02.ino 
  *
  * Modified by Rchard Hosking Dec 2024
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

// supply power to US transducer via a pin on the arduino
#define US_power 12
// RTC pin
#define RTC 2
// Serial port pins
#define Tx 10
#define Rx 11

// Distance variable
float distance;
// Baud rate for serial port
long BAUD = 9600;
// Inverse flag for serial port
bool inverse = false;

// Declare instance of US sensor
USsensor sensor(Tx,Rx, inverse);

// ISR (Interrupt service routine) for RTC
// Must have no parameters and no return
// Cannot call Class members?
void serviceRTC(void){
  //Serial.println("RTC interrupt");
  }

void setup(){
  // Start serial port to board for debugging purposes
 Serial.begin(9600);
 delay(200);
 Serial.println("Serial port active");

// Set up pins - hardware interrupt on pin 2 for RTC
 pinMode(RTC, INPUT);
 attachInterrupt(digitalPinToInterrupt(RTC), serviceRTC, CHANGE);
 
// "Tx" is the output from the transducer
// "Rx" sets mode - when high the transducer outputs a "processed" value 
// when low it outputs raw data
 pinMode(Tx, INPUT);
 pinMode(Rx, OUTPUT);
 // Set transducer mode to "processing"
 digitalWrite(Rx, HIGH);
  // Looks like this transducer will only work at 9600 baud
  // ser is a SoftwareSerial object in sensor
 sensor.begin(BAUD); 
 delay(300);
 if (sensor.ser.isListening()){
   Serial.println("Sensor listening");
 }
 else{
   Serial.println("Sensor not listening");
 }
} 


void loop()
{
    sensor.receiveFrame();
    sensor.measure();
    if (sensor.last_operate_status == STA_OK){
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
    
    sensor.clearBuffer();
  
  
delay(1000);


}




  