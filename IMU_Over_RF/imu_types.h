#ifndef _IMU_TYPES_H
#define _IMU_TYPES_H

#include "inttypes.h"

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
  float time, p1, p2;
  int chk;
} buffer_t;

void quat_ident(quat_t* out);

float quat_lenSq(quat_t* q);

float quat_len(quat_t* q);

void quat_inv(quat_t* q, quat_t* out);

void quat_norm(quat_t* q, quat_t* out);

void quat_mult(quat_t* a, quat_t* b, quat_t* out);

void quat_from_axis_angle(vec3_t* axis, float angle, quat_t* out);

void vec3_add(vec3_t* a, vec3_t* b, vec3_t* out);

void vec3_sub(vec3_t* a, vec3_t* b, vec3_t* out);

void vec3_mult(vec3_t* a, float b, vec3_t* out);

float vec3_dot(vec3_t* a, vec3_t* b);

void vec3_cross(vec3_t* a, vec3_t* b, vec3_t* out);

float vec3_ang(vec3_t* a, vec3_t* b);

float vec3_len(vec3_t* v);

float vec3_lenSq(vec3_t* v);

void vec3_norm(vec3_t* v, vec3_t* out);

void vec3_transf(vec3_t* v, quat_t* q, vec3_t* out);

void calc_tilt_correction(vec3_t* acc, vec3_t* est, vec3_t* out);

#endif
