#ifndef _IMU_TYPES_H
#define _IMU_TYPES_H

typedef struct
{
  float x, y, z, w;
} quat_t;

typedef struct
{
  float x, y, z;
} vec3_t;

typedef struct
{
  quat_t q;
  vec3_t a;
  vec3_t m;
  int16_t f1, f2, f3, f4;
  float p0, p1, p2;
  int chk;
} buffer_t;

#endif
