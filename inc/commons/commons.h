#ifndef COMMONS_H
#define COMMONS_H

#include <stdint.h>
#include <math.h>
#define UNUSED(x) ((void)x)

#define deg2rad_coeff ((float)M_PI / 180.0f)
#define rad2deg_coeff (180.0f / (float)M_PI)

///////////////////////////////////////////////////////
typedef struct {
  float ax, ay, az;
} AccelerometerData;

typedef struct {
  float gx, gy, gz;
} GyroscopeData;

typedef struct {
  float mx, my, mz;
} MagnetometerData;

typedef struct {
  AccelerometerData acc;
  GyroscopeData gyro;
} IMUData;

typedef struct {
  IMUData imu;
  MagnetometerData mag;
} AHRSData;
///////////////////////////////////////////////////////

char *int32ToStr(char *buff, int32_t val, uint8_t buff_len);
char *uint32ToStr(char *buff, uint32_t val, uint8_t buff_len);

char* ftoa(float f, char *buf, int precision);
void waitTicks(int ticks);
///////////////////////////////////////////////////////
#endif // COMMONS_H
