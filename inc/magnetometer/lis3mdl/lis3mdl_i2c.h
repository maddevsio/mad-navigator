#ifndef LIS3MDL_I2C_H
#define LIS3MDL_I2C_H


#include <stdbool.h>
#include <stdint.h>
#include "stm32f10x.h"

#define LIS3MDL_ADDRESS_HIGH 0x1e
#define LIS3MDL_ADDRESS_LOW 0x1c

#define LIS3MDL_PERFORMANCE_LOW_POWER   0x00
#define LIS3MDL_PERFORMANCE_MEDIUM      0x01
#define LIS3MDL_PERFORMANCE_HIGH        0x02
#define LIS3MDL_PERFORMANCE_ULTRA_HIGH  0x03

#define LIS3MDL_PRECISION_LP

#define LIS3MDL_DATA_RATE_0_625_HZ  0x00
#define LIS3MDL_DATA_RATE_1_25_HZ   0x01
#define LIS3MDL_DATA_RATE_2_5_HZ    0x02
#define LIS3MDL_DATA_RATE_5_HZ      0x03
#define LIS3MDL_DATA_RATE_10_HZ     0x04
#define LIS3MDL_DATA_RATE_20_HZ     0x05
#define LIS3MDL_DATA_RATE_40_HZ     0x06
#define LIS3MDL_DATA_RATE_80_HZ     0x07

#define LIS3MDL_MODE_CONTINUOUS   0x00
#define LIS3MDL_MODE_SINGLE       0x01
#define LIS3MDL_MODE_POWER_DOWN   0x03

#define LIS3MDL_SCALE_4_GAUSS   0x00
#define LIS3MDL_SCALE_8_GAUSS   0x01
#define LIS3MDL_SCALE_12_GAUSS  0x02
#define LIS3MDL_SCALE_16_GAUSS  0x03

#define LIS3MDL_AXIS_X 0
#define LIS3MDL_AXIS_Y 1
#define LIS3MDL_AXIS_Z 2

#define LIS3MDL_STATUS_ZYXOR  0x80
#define LIS3MDL_STATUS_ZOR    0x40
#define LIS3MDL_STATUS_YOR    0x20
#define LIS3MDL_STATUS_XOR    0x10
#define LIS3MDL_STATUS_ZYXDA  0x08
#define LIS3MDL_STATUS_ZDA    0x04
#define LIS3MDL_STATUS_YDA    0x02
#define LIS3MDL_STATUS_XDA    0x01

#define LIS3MDL_DEVICE_ID 0x3d

typedef struct {  
  uint8_t address;
  uint16_t scale;
  int16_t min[3];
  int16_t max[3];
} lis3mdl_t;

typedef int8_t lis3mdl_err_t;

lis3mdl_err_t LIS3MDL_setup(lis3mdl_t* lis3mdl, uint8_t address);
lis3mdl_err_t LIS3MDL_reset(lis3mdl_t* lis3mdl);

void LIS3MDL_clearMinMax(lis3mdl_t* lis3mdl);
void LIS3MDL_setMinMax(lis3mdl_t* lis3mdl, uint8_t axis, int16_t min, int16_t max);

float LIS3MDL_currentGain(const lis3mdl_t *lis3mdl);

void LIS3MDL_enableTemperature(lis3mdl_t* lis3mdl, bool enable);
void LIS3MDL_enableODR(lis3mdl_t *lis3mdl, bool enable);

void LIS3MDL_setPerformance(lis3mdl_t* lis3mdl, uint8_t performance);
void LIS3MDL_setDataRate(lis3mdl_t* lis3mdl, uint8_t dataRate);
void LIS3MDL_setMode(lis3mdl_t* lis3mdl, uint8_t mode);
void LIS3MDL_setScale(lis3mdl_t* lis3mdl, uint8_t scale);

void LIS3MDL_read3AxisSync(lis3mdl_t* lis3mdl, int16_t value[3]);
void LIS3MDL_readAxis(lis3mdl_t* lis3mdl, uint8_t axis, int16_t* value);

void LIS3MDL_readTemperature(lis3mdl_t* lis3mdl, int16_t* value);
lis3mdl_err_t LIS3MDL_readDeviceId(lis3mdl_t* lis3mdl, uint8_t* deviceId);
void LIS3MDL_readStatus(lis3mdl_t* lis3mdl, uint8_t* status);

#endif // LIS3MDL_I2C_H
