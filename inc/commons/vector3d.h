#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <stdint.h>

typedef struct vector3d {
  float x, y, z, len;
} vector3d_t;

vector3d_t vector_new(float x, float y, float z);
void vector_normalize(vector3d_t *v3d);

float vector_rotationSign(const vector3d_t *a, const vector3d_t *b);
float vector_flatCos(const vector3d_t *a, const vector3d_t *b);

#endif // VECTOR3D_H
