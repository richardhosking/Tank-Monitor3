#include <SoftwareSerial.h>
#include "uSsensor.h"

// Constructor implicitly called when class is instantiated

USsensor::USsensor(int TX, int RX, bool inverse)
{
  distance_max = DISTANCE_MAX;
  distance_min = DISTANCE_MIN;
  last_operate_status = STA_OK;
  distance = 0;
  sum = 0;
  ser = SoftwareSerial(TX, RX,inverse);

}
// Open serial port
void USsensor::begin(int BAUD){
  ser.begin(BAUD);
  delay(200);
  if (ser.available() == 0) {
    last_operate_status = STA_OK;
  }
  else
  {
    last_operate_status = STA_ERR_SERIAL;
  }
}
/* the serial port seems to store the frame header in byte 3 with the current code ??why
 and then the other data bytes Data High Byte 0 Data low Byte 1 and checksum byte 2
  in a 4 byte frame */
/* Checksum is the last byte in the frame from the transducer
  The checksum is the sum of the first three bytes of the frame
  The sum is then bitwise ANDed with 0x00ff to get the checksum */

int USsensor::check_sum(unsigned char checksum[])
{
  return (checksum[3] + checksum[0] + checksum[1])&0x00ff;
}

/* Measure distance from transducer to flat plane surface
  The distance is stored in the first two bytes of the frame
  The distance is the high byte multiplied by 256 plus the low byte
  The distance is then checked against the distance limits and the checksum
*/
int USsensor::measure()
{  
  if(data[3]==0xff)
  {
    sum = check_sum(data);
      if (sum != data[2])
      {
        last_operate_status = STA_ERR_CHECKSUM;
      }
      else
      {
        distance = data[0]*256 + data[1];
        last_operate_status = STA_OK;
      }
      if (distance > distance_max)
      {
        last_operate_status = STA_ERR_CHECK_OUT_LIMIT;
        distance = distance_max;
      }
      else if (distance < distance_min)
      {
        last_operate_status = STA_ERR_CHECK_LOW_LIMIT;
        distance = distance_min;
      }
    }
    else
    {
      last_operate_status = STA_ERR_DATA;
    }
    newData = false;
    
    if (last_operate_status == STA_OK)
    {
      return distance;
    }
    else
    {
      return -1;
    }
    
  }

int USsensor::get_last_operate_status()
{
  return last_operate_status;
}

/* There does not seem to be a way to clear the serial buffer in the SoftwareSerial library
  This function reads the serial buffer until it is empty and clears the newData flag
*/
void USsensor::clearBuffer()
{
  while(ser.available() > 0)
  {
    ser.read();
  }
  newData = false;
}

/* Receive a frame from the transducer
  The frame is 4 bytes long
  The first byte is the frame header
  The next two bytes are the distance
  The last byte is the checksum
  The frame header is 0xFF
  */
void USsensor::receiveFrame(){

    int ndx = 0;       
    if (ser.available() > 0 && newData == false) {        
        do {
          for (int i = 0; i < 4; i++){
            data[i] = ser.read();            
            }
        } while (ser.read() == startFrame);
    }
          else{
            last_operate_status = STA_ERR_DATA;
          }
      // Set flag to indicate new data is available       
      newData = true;
    }

