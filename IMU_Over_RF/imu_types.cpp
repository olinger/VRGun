#include "imu_types.h"

#include "inttypes.h"
#include "math.h"

const float TOLERANCE = 0.00001;

void quat_ident(quat_t* out)
{
  out->x = 0;
  out->y = 0;
  out->z = 0;
  out->w = 1;
}

float quat_lenSq(quat_t* q)
{
  return q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w;
}

float quat_len(quat_t* q)
{
  return sqrt(quat_lenSq(q));
}

void quat_inv(quat_t* q, quat_t* out)
{
  float lSq = quat_lenSq(q);
  if (lSq != 0.0)
  {
    float i = 1.0 / lSq;
    out->x = q->x * -i;
    out->y = q->y * -i;
    out->z = q->z * -i;
    out->w = q->w * i;
  }
  else
    *out = *q;
}

void quat_norm(quat_t* q, quat_t* out)
{
  float len = quat_len(q);
  
  out->x = q->x / len;
  out->y = q->y / len;
  out->z = q->z / len;
  out->w = q->w / len;
}

void quat_mult(quat_t* a, quat_t* b, quat_t* out)
{
  out->x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
  out->y = a->w * b->y + a->x * b->z + a->y * b->w + a->z * b->x;
  out->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
  out->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
}

void quat_from_axis_angle(vec3_t* axis, float angle, quat_t* out)
{
  if (vec3_lenSq(axis) == 0)
  {
    quat_ident(out);
    return;
  }
    
  quat_ident(out);
  
  angle *= 0.5;
  float angS = sin(angle);
  float angC = cos(angle);
  vec3_t axis_n;
  vec3_norm(axis, &axis_n);
  
  out->x = axis_n.x * angS;
  out->y = axis_n.y * angS;
  out->z = axis_n.z * angS;
  out->w = angC;
  
  quat_norm(out, out);
}

void vec3_add(vec3_t* a, vec3_t* b, vec3_t* out)
{
  out->x = a->x + b->x;
  out->y = a->y + b->y;
  out->z = a->z + b->z;
}

void vec3_sub(vec3_t* a, vec3_t* b, vec3_t* out)
{
  out->x = a->x - b->x;
  out->y = a->y - b->y;
  out->z = a->z - b->z;
}

void vec3_mult(vec3_t* a, float b, vec3_t* out)
{
  out->x = a->x * b;
  out->y = a->y * b;
  out->z = a->z * b;
}

float vec3_dot(vec3_t* a, vec3_t* b)
{
  return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vec3_cross(vec3_t* a, vec3_t* b, vec3_t* out)
{
  out->x = a->y * b->z - a->z * b->y;
  out->y = a->z * b->x - a->x * b->z;
  out->z = a->x * b->y - a->y * b->x;
}

float vec3_ang(vec3_t* a, vec3_t* b)
{
  float d = vec3_dot(a, b);
  float al = vec3_len(a);
  float bl = vec3_len(b);
  
  return acos(d / (al * bl));
}

float vec3_len(vec3_t* v)
{
  return sqrt(vec3_lenSq(v));
}

float vec3_lenSq(vec3_t* v)
{
  return v->x * v->x + v->y * v->y + v->z * v->z;
}

void vec3_norm(vec3_t* v, vec3_t* out)
{
  float len = vec3_len(v);
  out->x = v->x / len;
  out->y = v->y / len;
  out->z = v->z / len;
}

void vec3_transf(vec3_t* v, quat_t* q, vec3_t* out)
{
  vec3_t xyz, tmp, tmp2;
  xyz.x = q->x;
  xyz.y = q->y;
  xyz.z = q->z;
  
  vec3_cross(&xyz, v, &tmp);
  vec3_mult(v, q->w, &tmp2);
  vec3_add(&tmp, &tmp2, &tmp);
  vec3_cross(&xyz, &tmp, &tmp);
  vec3_mult(&tmp, 2, &tmp);
  vec3_add(v, &tmp, out);
}

void calc_tilt_correction(vec3_t* acc, vec3_t* est, vec3_t* out)
{
  vec3_norm(acc, acc);
  vec3_norm(est, est);
  
  vec3_t corrected;
  vec3_cross(acc, est, &corrected);
  float cosErr = vec3_dot(acc, est);
  vec3_mult(&corrected, sqrt(2 / (1 + cosErr + TOLERANCE)), out);
}
