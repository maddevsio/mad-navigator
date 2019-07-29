#ifndef QUATERNION_H
#define QUATERNION_H

#include <stdint.h>
#include "commons/vector3d.h"

typedef struct quaternion {
  float w, x, y, z;
} quaternion_t;

quaternion_t quaternion_new_wxyz(float w, float x, float y, float z);
quaternion_t quaternion_new_vec(const vector3d_t *rv, float angleRads);
vector3d_t quaternion_transform_vec(const quaternion_t *q, const vector3d_t *v);

float quaternion_pitch(const quaternion_t *q);
float quaternion_roll(const quaternion_t *q);
float quaternion_yaw(const quaternion_t *q);
#endif // QUATERNION_H
