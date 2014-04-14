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
#include "imu_types.h"
#include "math.h"

//RF24 stuff
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

const float GYRO_SCALE_DIV = 14.375;
const float DEG_2_RAD = (PI / 180);

const float ACC_SCALE_DIV = 256;
const float GRAVITY = 9.806;

const float GAIN = 0.05;
const float TOLERANCE = 0.00001;

const float MILLIS = 0.01;

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

quat_t quat_ident()
{
  quat_t q;
  q.x = 0;
  q.y = 0;
  q.z = 0;
  q.w = 1;
  
  return q;
}

float quat_len(quat_t q)
{
  return sqrt(quat_lenSq(q));
}

float quat_lenSq(quat_t q)
{
  return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

quat_t quat_inv(quat_t q)
{
  float lSq = quat_lenSq(q);
  if (lSq != 0.0)
  {
    float i = 1.0 / lSq;
    quat_t nq;
    nq.x = q.x * -i;
    nq.y = q.y * -i;
    nq.z = q.z * -i;
    nq.w = q.w * i;
    return nq;
  }
  else
    return q;
}

quat_t quat_norm(quat_t q)
{
  quat_t r;
  float len = quat_len(q);
  
  r.x = q.x / len;
  r.y = q.y / len;
  r.z = q.z / len;
  r.w = q.w / len;
  
  return r;
}

quat_t quat_mult(quat_t a, quat_t b)
{
  quat_t r;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y + a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  
  return r;
}

quat_t quat_from_axis_angle(vec3_t axis, float angle)
{
  if (vec3_lenSq(axis) == 0)
    return quat_ident();
    
  quat_t r = quat_ident();
  
  angle *= 0.5;
  float angS = sin(angle);
  float angC = cos(angle);
  axis = vec3_norm(axis);
  
  r.x = axis.x * angS;
  r.y = axis.y * angS;
  r.z = axis.z * angS;
  r.w = angC;
  
  return quat_norm(r);
}

vec3_t vec3_add(vec3_t a, vec3_t b)
{
  vec3_t r;
  r.x = a.x + b.x;
  r.y = a.y + b.y;
  r.z = a.z + b.z;
  
  return r;
}

vec3_t vec3_sub(vec3_t a, vec3_t b)
{
  vec3_t r;
  r.x = a.x - b.x;
  r.y = a.y - b.y;
  r.z = a.z - b.z;
  
  return r;
}

vec3_t vec3_mult(vec3_t a, float b)
{
  vec3_t r;
  r.x = a.x * b;
  r.y = a.y * b;
  r.z = a.z * b;
  
  return r;
}

float vec3_dot(vec3_t a, vec3_t b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

vec3_t vec3_cross(vec3_t a, vec3_t b)
{
  vec3_t r;
  r.x = a.y * b.z - a.z * b.y;
  r.y = a.z * b.x - a.x * b.z;
  r.z = a.x * b.y - a.y * b.x;
  
  return r;
}

float vec3_ang(vec3_t a, vec3_t b)
{
  float d = vec3_dot(a, b);
  float al = vec3_len(a);
  float bl = vec3_len(b);
  
  return acos(d / (al * bl));
}

float vec3_len(vec3_t v)
{
  return sqrt(vec3_lenSq(v));
}

float vec3_lenSq(vec3_t v)
{
  return v.x * v.x + v.y * v.y + v.z * v.z;
}

vec3_t vec3_norm(vec3_t v)
{
  float len = vec3_len(v);
  vec3_t r;
  r.x = v.x / len;
  r.y = v.y / len;
  r.z = v.z / len;
  
  return r;
}

vec3_t vec3_transf(vec3_t v, quat_t q)
{
  vec3_t xyz, tmp, tmp2;
  xyz.x = q.x;
  xyz.y = q.y;
  xyz.z = q.z;
  
  tmp = vec3_cross(xyz, v);
  tmp2 = vec3_mult(v, q.w);
  tmp = vec3_add(tmp, tmp2);
  tmp = vec3_cross(xyz, tmp);
  tmp = vec3_mult(tmp, 2);
  return vec3_add(v, tmp);
}

vec3_t calc_tilt_correction(vec3_t acc, vec3_t est)
{
  acc = vec3_norm(acc);
  est = vec3_norm(est);
  
  vec3_t corrected = vec3_cross(acc, est);
  float cosErr = vec3_dot(acc, est);
  return vec3_mult(corrected, sqrt(2 / (1 + cosErr + TOLERANCE)));
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    
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
    b.q = quat_ident();
}

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
    
    vec3_t acc;
    acc.x = ax / ACC_SCALE_DIV * GRAVITY;
    acc.y = az / ACC_SCALE_DIV * GRAVITY;
    acc.z = ay / ACC_SCALE_DIV * GRAVITY;
    
    quat_t qInv = quat_inv(b.q);
    vec3_t up;
    up.x = 0;
    up.y = 1;
    up.z = 0;
    up = vec3_transf(up, qInv);
    
    vec3_t gyr;
    gyr.x = gx / GYRO_SCALE_DIV * DEG_2_RAD;
    gyr.y = gy / GYRO_SCALE_DIV * DEG_2_RAD;
    gyr.z = gz / GYRO_SCALE_DIV * DEG_2_RAD;
    gyr = vec3_sub(gyr, gyroOff);
    
    vec3_t down;
    down.x = 0;
    down.y = 1;
    down.z = 0;
    down = vec3_transf(down, qInv);
    down = vec3_mult(down, GRAVITY);
    acc = vec3_sub(acc, down);
    
    float spikeThreshold = 0.01;
    float gravityThreshold = 0.1;
    float proportionalGain = 5 * GAIN;
    float integralGain = 0.0125f;
    
    vec3_t tilt_correct = calc_tilt_correction(acc, up);
    
    float tiltAng = vec3_ang(up, acc);
    tiltFilter_add(tiltAng);
    if (tiltAng > tiltFilter_mean() + spikeThreshold)
      proportionalGain = integralGain = 0;
    
    if (abs(vec3_len(acc) / GRAVITY - 1) > gravityThreshold)
      integralGain = 0;
      
    gyr = vec3_add(gyr, vec3_mult(tilt_correct, proportionalGain));
    gyroOff = vec3_sub(gyroOff, vec3_mult(tilt_correct, integralGain * MILLIS));
    
    float ang = vec3_len(gyr) * MILLIS;
    vec3_t axis = vec3_norm(gyr);
    
    if (ang > 0)
      b.q = quat_mult(b.q, quat_from_axis_angle(axis, ang));
      
    vec3_t mag;
    mag.x = mx * DEG_2_RAD;
    mag.y = my * DEG_2_RAD;
    mag.z = mz * DEG_2_RAD;
      
    //b.q = quat_norm(b.q);
    b.a = acc;
    b.m = mag;
    b.f1 = analogRead(23);
    b.f2 = analogRead(22);
    b.f3 = analogRead(21);
    b.f4 = analogRead(20);
    b.chk = 1337;
    n = RawHID.send(&b, 2);
}
