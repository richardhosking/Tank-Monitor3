/*
@file USsensor.h
Heavily modified from the Python code for the DFRobot sensor as the Arduino C++ is not well documented
Richard Hosking Apr- Dec 2024
@copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT License (MIT)
@author      Arya(xue.peng@dfrobot.com)
@version     V1.0
@date        2021-08-30
@url https://github.com/DFRobot/DFRobot_RaspberryPi_A02YYUW
*/

#ifndef USsensor_h
#define USsensor_h

#include <Arduino.h>
#include <SoftwareSerial.h>

// Define distance limits for transducer 280 to 7500mm 
#define DISTANCE_MAX 7500
#define DISTANCE_MIN 280

// Status codes - "last_operate_status" is defined in inherited SoftwareSerial class
enum status_t
{
  STA_OK = 0x00,
  STA_ERR_CHECKSUM = 0x01,
  STA_ERR_SERIAL = 0x02,
  STA_ERR_CHECK_OUT_LIMIT = 0x03,
  STA_ERR_CHECK_LOW_LIMIT = 0x04,
  STA_ERR_DATA = 0x05
};

// rebuild class from Python code - inherit SoftwareSerial library
class USsensor : public SoftwareSerial {
  public:
  
  USsensor(int RX, int TX, bool inverse);
  void begin(int BAUD);
  int get_last_operate_status();
  int measure();
  int check_sum(unsigned char checksum[]);
  void clearBuffer();
  void receiveFrame();
  
  int distance_max;
  int distance_min;
  int last_operate_status;
  int distance;
  int sum;
  SoftwareSerial ser;

 private:
  // Array to hold data from serial port
  // Byte[3] = 0xFF if valid data 
  // Byte[0] = high byte of distance
  // Byte[1] = low byte of distance
  // Byte[2] = checksum
  
  unsigned char data[4]={};
  unsigned char startFrame = 0xFF;
  unsigned char rc;
  bool newData = false;
};

#endif