/* Basic Raw HID Example
   Teensy can send/receive 64 byte packets with a
   dedicated program running on a PC or Mac.

   You must select Raw HID from the "Tools > USB Type" menu

   Optional: LEDs should be connected to pins 0-7,
   and analog signals to the analog inputs.

   This example code is in the public domain.
*/
#include "Wire.h"

#include "imu_types.h"

// I2Cdev and ADXL345 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"
#include "math.h"

const float GYRO_SCALE_DIV = 14.375;
const float DEG_2_RAD = (PI / 180);

const float ACC_SCALE_DIV = 256;
const float GRAVITY = 9.806;

const float GAIN = 0.05;

const float MS_2_S = 0.001;
const float US_2_S = 0.000001;

elapsedMicros elapsedUs;

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

vec3_t gyroOff;

float tiltFilter[1000];
int tiltFilterInd = 0;
int tiltFilterCount = 0;
buffer_t b;

int asdf;

// A9-6 are for reading flex sensors
// D3-6 are for writing to vibes

//RF24 radio(10, 12);
//const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

void tiltFilter_add(float ang)
{
  tiltFilter[tiltFilterInd++] = ang;
  if (tiltFilterInd >= 1000)
    tiltFilterInd = 0;
    
  if (tiltFilterCount < 1000)
    tiltFilterCount++;
}

float tiltFilter_mean()
{
  float sum = 0;
  for (int i = 0; i < tiltFilterCount; i++)
    sum += tiltFilter[i];
  return sum / tiltFilterCount;
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    
    pinMode(20, INPUT);
    pinMode(21, INPUT);
    pinMode(22, INPUT);
    pinMode(23, INPUT);
    
    /*analogWrite(3, 0xff);
    analogWrite(4, 0xff);
    analogWrite(5, 0xff);
    analogWrite(6, 0xff);*/
    
    accel.initialize();
    while (!accel.testConnection()){delay(1);}
    
    //accel.setRange(ADXL345_RANGE_8G);
    
    mag.initialize();
    while (!mag.testConnection()){delay(1);}
    
    gyro.initialize();
    while (!gyro.testConnection()){delay(1);}
    
    vec3_t v;
    v.x = v.y = v.z = 0;
    for (int i = 0; i < 100; i++)
    {
      gyro.getRotation(&gx, &gy, &gz);
      v.x += (float)gx / GYRO_SCALE_DIV * DEG_2_RAD;
      v.y += (float)gy / GYRO_SCALE_DIV * DEG_2_RAD;
      v.z += (float)gz / GYRO_SCALE_DIV * DEG_2_RAD;
    }
    
    v.x /= 100;
    v.y /= 100;
    v.z /= 100;
    
    gyroOff = v;
    quat_ident(&b.q);
    elapsedUs = 0;
}

unsigned int packetCount = 0;
int n;
void loop() 
{
  float time = elapsedUs * US_2_S;
  elapsedUs = 0;
  //if (msUntilNextSend > 100)
  //{
    //msUntilNextSend = 0;
    accel.getAcceleration(&ax, &ay, &az);
    mag.getHeading(&mx, &my, &mz);
    gyro.getRotation(&gx, &gy, &gz);
    
    vec3_t acc;
    acc.x = ax / ACC_SCALE_DIV * GRAVITY;
    acc.y = az / ACC_SCALE_DIV * GRAVITY;
    acc.z = ay / ACC_SCALE_DIV * GRAVITY;
    
    quat_t qInv;
    quat_inv(&b.q, &qInv);
    vec3_t up;
    up.x = 0;
    up.y = 1;
    up.z = 0;
    vec3_transf(&up, &qInv, &up);
    
    vec3_t gyr;
    gyr.x = gx / GYRO_SCALE_DIV * DEG_2_RAD;
    gyr.y = gy / GYRO_SCALE_DIV * DEG_2_RAD;
    gyr.z = gz / GYRO_SCALE_DIV * DEG_2_RAD;
    vec3_sub(&gyr, &gyroOff, &gyr);
    
    vec3_t down;
    down.x = 0;
    down.y = 1;
    down.z = 0;
    vec3_transf(&down, &qInv, &down);
    vec3_mult(&down, GRAVITY, &down);
    vec3_sub(&acc, &down, &down);
    
    float spikeThreshold = 0.01;
    float gravityThreshold = 0.1;
    float proportionalGain = 5 * GAIN;
    float integralGain = 0.0125f;
    
    vec3_t tilt_correct;
    calc_tilt_correction(&acc, &up, &tilt_correct);
    
    float tiltAng = vec3_ang(&up, &acc);
    tiltFilter_add(tiltAng);
    if (tiltAng > tiltFilter_mean() + spikeThreshold)
      proportionalGain = integralGain = 0;
    
    if (abs(vec3_len(&acc) / GRAVITY - 1) > gravityThreshold)
      integralGain = 0;
    
    vec3_t tmp;
    vec3_mult(&tilt_correct, proportionalGain, &tmp);
    vec3_add(&gyr, &tmp, &gyr);
    
    vec3_mult(&tilt_correct, integralGain * time, &tmp);
    vec3_sub(&gyroOff, &tmp, &gyroOff);
    
    float ang = vec3_len(&gyr) * time;
    vec3_t axis;
    vec3_norm(&gyr, &axis);
    
    if (ang > 0)
    {
      quat_t qtmp;
      quat_from_axis_angle(&axis, ang, &qtmp);
      quat_mult(&b.q, &qtmp, &b.q);
    }
      
    vec3_t mag;
    mag.x = mx * DEG_2_RAD;
    mag.y = my * DEG_2_RAD;
    mag.z = mz * DEG_2_RAD;
      
    quat_norm(&b.q, &b.q);
    b.a = acc;
    b.m = mag;
    b.f1 = analogRead(23);
    b.f2 = analogRead(22);
    b.f3 = analogRead(21);
    b.f4 = analogRead(20);
    b.time = time;
    b.chk = 1337;
    n = RawHID.send(&b, 1);
    //delay(1);
}
