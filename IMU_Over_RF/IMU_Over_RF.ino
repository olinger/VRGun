 /* Basic Raw HID Example
   Teensy can send/receive 64 byte packets with a
   dedicated program running on a PC or Mac.

   You must select Raw HID from the "Tools > USB Type" menu

   Optional: LEDs should be connected to pins 0-7,
   and analog signals to the analog inputs.

   This example code is in the public domain.
*/
#include "Wire.h"

// I2Cdev and ADXL345 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

//RF24 stuff
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// class default I2C address is 0x53
// specific I2C addresses may be passed as a parameter here
// ALT low = 0x53 (default for SparkFun 6DOF board)
// ALT high = 0x1D
ADXL345 accel;

int16_t ax, ay, az;

// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;

int16_t mx, my, mz;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun 6DOF board)
// AD0 high = 0x69 (default for SparkFun ITG-3200 standalone board)
ITG3200 gyro;

int16_t gx, gy, gz;

int asdf;

//RF24 radio(10, 12);
//const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    accel.initialize();
    while (!accel.testConnection()){delay(1);}
    
    //accel.setRange(ADXL345_RANGE_8G);
    
    mag.initialize();
    while (!mag.testConnection()){delay(1);}
    
    gyro.initialize();
    while (!gyro.testConnection()){delay(1);}
    
    //gyro.setFullScaleRange(3);
    
    /*delay(1000);
    gyro.init(ITG3200_ADDR_AD0_LOW);*/
    
  //  radio.begin();
    //radio.setRetries(15, 15);
    
    //radio.openWritingPipe(pipes[0]);
    //radio.openReadingPipe(1, pipes[1]);
}

// RawHID packets are always 64 bytes
byte buffer[64];
elapsedMillis msUntilNextSend;
unsigned int packetCount = 0;
int n;
void loop() 
{    
  //if (msUntilNextSend > 100)
  //{
    //msUntilNextSend = 0;
    accel.getAcceleration(&ax, &ay, &az);
    mag.getHeading(&mx, &my, &mz);
    gyro.getRotation(&gx, &gy, &gz);
    
    /*if (gyro.isRawDataReady())
    {
      gyro.readGyro(&gx, &gy, &gz);
    }*/

    buffer[0] = 0xAB;
    buffer[1] = 0xCD;
    
    buffer[2] = lowByte(ax);
    buffer[3] = highByte(ax);
    buffer[4] = lowByte(ay);
    buffer[5] = highByte(ay);
    buffer[6] = lowByte(az);
    buffer[7] = highByte(az);
    
    buffer[8] = lowByte(mx);
    buffer[9] = highByte(mx);
    buffer[10] = lowByte(my);
    buffer[11] = highByte(my);
    buffer[12] = lowByte(mz);
    buffer[13] = highByte(mz);
    
    /*int16_t gxi = (int16_t)gx;
    int16_t gyi = (int16_t)gy;
    int16_t gzi = (int16_t)gz;*/
    
    buffer[14] = lowByte(gx);
    buffer[15] = highByte(gx);
    buffer[16] = lowByte(gy);
    buffer[17] = highByte(gy);
    buffer[18] = lowByte(gz);
    buffer[19] = highByte(gz);
    // fill the rest with zeros
    for (int i=20; i<64; i++) {
      buffer[i] = 0;
    }
    // and put a count of packets sent at the end
    buffer[62] = lowByte(packetCount);
    buffer[63] = highByte(packetCount);
    packetCount = 1337;
    // actually send the packet
    n = RawHID.send(buffer, 2);
    //digitalWrite(13, HIGH);
  //}
  
  //radio.stopListening();
  
  //bool ok = radio.write(buffer, 64);
  
  //if (!ok)
    //Serial.write("SEND FAILED\n");
    
  //radio.startListening();
}
