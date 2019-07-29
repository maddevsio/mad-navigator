#include "magnetometer/lis3mdl/lis3mdl_i2c.h"
#include <inttypes.h>
#include <limits.h>
#include "i2c1/i2c1.h"

#define LIS3MDL_REG_WHO_AM_I     0x0F
#define LIS3MDL_REG_CTL_1        0x20
#define LIS3MDL_REG_CTL_2        0x21
#define LIS3MDL_REG_CTL_3        0x22
#define LIS3MDL_REG_CTL_4        0x23
#define LIS3MDL_REG_STATUS       0x27
#define LIS3MDL_REG_OUT_X_L      0x28
#define LIS3MDL_REG_OUT_X_H      0x29
#define LIS3MDL_REG_OUT_Y_L      0x2A
#define LIS3MDL_REG_OUT_Y_H      0x2B
#define LIS3MDL_REG_OUT_Z_L      0x2C
#define LIS3MDL_REG_OUT_Z_H      0x2D
#define LIS3MDL_REG_OUT_TEMP_L   0x2E
#define LIS3MDL_REG_OUT_TEMP_H   0x2F

#define LIS3MDL_REG_CTL_1_TEMP_EN 0x80
#define LIS3MDL_REG_CTL_1_ODR_EN 0x02
#define LIS3MDL_REG_CTL_2_RESET 0x04

static const uint16_t LIS3MDLGAUSS_TO_SCALE[] = { 4, 8, 12, 16 };

static lis3mdl_err_t _LIS3MDL_init(lis3mdl_t* lis3mdl);
static lis3mdl_err_t _LIS3MDL_readRegister(lis3mdl_t* lis3mdl, uint8_t reg, uint8_t* value);
static lis3mdl_err_t _LIS3MDL_writeRegister(lis3mdl_t* lis3mdl, uint8_t reg, uint8_t data, uint8_t mask);
static void _LIS3MDL_readRegister_int16(lis3mdl_t* lis3mdl, uint8_t lowAddr, uint8_t highAddr, int16_t* value);

lis3mdl_err_t LIS3MDL_setup(lis3mdl_t* lis3mdl,
                            uint8_t address) {
  lis3mdl->address = address;
  LIS3MDL_clearMinMax(lis3mdl);
  return _LIS3MDL_init(lis3mdl);
}
///////////////////////////////////////////////////////

void LIS3MDL_clearMinMax(lis3mdl_t* lis3mdl) {
  for (int axis = 0; axis < 3; axis++) {
    lis3mdl->min[axis] = INT16_MAX;
    lis3mdl->max[axis] = INT16_MIN;
  }
}
///////////////////////////////////////////////////////

void LIS3MDL_setMinMax(lis3mdl_t* lis3mdl,
                       uint8_t axis,
                       int16_t min,
                       int16_t max) {
  lis3mdl->min[axis] = min;
  lis3mdl->max[axis] = -max;
}
///////////////////////////////////////////////////////

lis3mdl_err_t LIS3MDL_reset(lis3mdl_t* lis3mdl) {
  _LIS3MDL_writeRegister(lis3mdl, LIS3MDL_REG_CTL_2, LIS3MDL_REG_CTL_2_RESET, LIS3MDL_REG_CTL_2_RESET);
  return _LIS3MDL_init(lis3mdl);
}
///////////////////////////////////////////////////////

void LIS3MDL_enableTemperature(lis3mdl_t* lis3mdl,
                               bool enable) {
  _LIS3MDL_writeRegister(lis3mdl, LIS3MDL_REG_CTL_1, enable, LIS3MDL_REG_CTL_1_TEMP_EN);
}
///////////////////////////////////////////////////////

void LIS3MDL_setPerformance(lis3mdl_t* lis3mdl,
                            uint8_t performance) {
  _LIS3MDL_writeRegister(lis3mdl, LIS3MDL_REG_CTL_1, (uint8_t)(performance << 5), 0x60); // 0b01100000
  _LIS3MDL_writeRegister(lis3mdl, LIS3MDL_REG_CTL_4, (uint8_t)(performance << 2), 0x0c); // 0b00001100
}
///////////////////////////////////////////////////////

void LIS3MDL_setDataRate(lis3mdl_t* lis3mdl,
                         uint8_t dataRate) {
  _LIS3MDL_writeRegister(lis3mdl, LIS3MDL_REG_CTL_1, (uint8_t)(dataRate << 2), 0x1c); // 0b00011100
}
///////////////////////////////////////////////////////

void LIS3MDL_setMode(lis3mdl_t* lis3mdl,
                     uint8_t mode) {
  _LIS3MDL_writeRegister(lis3mdl, LIS3MDL_REG_CTL_3, (uint8_t)(mode << 0), 0x03); // 0b00000011
}
///////////////////////////////////////////////////////

void LIS3MDL_setScale(lis3mdl_t* lis3mdl,
                      uint8_t scale) {
  _LIS3MDL_writeRegister(lis3mdl, LIS3MDL_REG_CTL_2, (uint8_t)(scale << 5), 0x60); // 0b01100000
  lis3mdl->scale = LIS3MDLGAUSS_TO_SCALE[scale];
}
///////////////////////////////////////////////////////

void LIS3MDL_readAxis(lis3mdl_t* lis3mdl,
                      uint8_t axis,
                      int16_t* value) {
  uint8_t lowAddr, highAddr;
  lowAddr = highAddr = 0;
  switch (axis) {
    case LIS3MDL_AXIS_X:
      lowAddr = LIS3MDL_REG_OUT_X_L;
      highAddr = LIS3MDL_REG_OUT_X_H;
      break;
    case LIS3MDL_AXIS_Y:
      lowAddr = LIS3MDL_REG_OUT_Y_L;
      highAddr = LIS3MDL_REG_OUT_Y_H;
      break;
    case LIS3MDL_AXIS_Z:
      lowAddr = LIS3MDL_REG_OUT_Z_L;
      highAddr = LIS3MDL_REG_OUT_Z_H;
      break;    
  }
  _LIS3MDL_readRegister_int16(lis3mdl, lowAddr, highAddr, value);
}
///////////////////////////////////////////////////////

void LIS3MDL_readTemperature(lis3mdl_t* lis3mdl,
                             int16_t* value) {
  _LIS3MDL_readRegister_int16(lis3mdl, LIS3MDL_REG_OUT_TEMP_L, LIS3MDL_REG_OUT_TEMP_H, value);
}
///////////////////////////////////////////////////////

lis3mdl_err_t LIS3MDL_readDeviceId(lis3mdl_t* lis3mdl,
                                   uint8_t* deviceId) {
  return _LIS3MDL_readRegister(lis3mdl, LIS3MDL_REG_WHO_AM_I, deviceId);
}
///////////////////////////////////////////////////////

void LIS3MDL_readStatus(lis3mdl_t* lis3mdl,
                        uint8_t* status) {
  _LIS3MDL_readRegister(lis3mdl, LIS3MDL_REG_STATUS, status);
}
///////////////////////////////////////////////////////

lis3mdl_err_t _LIS3MDL_init(lis3mdl_t* lis3mdl) {
  uint8_t deviceId = 0;
  uint8_t ctl2 = 0;
  lis3mdl_err_t err = LIS3MDL_readDeviceId(lis3mdl, &deviceId);

  if (err)
    return err;

  if (deviceId != LIS3MDL_DEVICE_ID)
    return 1;

  _LIS3MDL_readRegister(lis3mdl, LIS3MDL_REG_CTL_2, &ctl2);
  lis3mdl->scale = LIS3MDLGAUSS_TO_SCALE[(ctl2 >> 5) & 0x03];
  return 0;
}
///////////////////////////////////////////////////////

void _LIS3MDL_readRegister_int16(lis3mdl_t* lis3mdl,
                                 uint8_t lowAddr,
                                 uint8_t highAddr,
                                 int16_t* value) {
  uint8_t low, high;
  _LIS3MDL_readRegister(lis3mdl, lowAddr, &low);
  _LIS3MDL_readRegister(lis3mdl, highAddr, &high);
  *value = (int16_t)(((uint16_t)high) << 8) | (uint16_t)low; //here parses two's complement because of ARM arch
}
///////////////////////////////////////////////////////

lis3mdl_err_t _LIS3MDL_readRegister(lis3mdl_t* lis3mdl,
                                    uint8_t reg,
                                    uint8_t* value) {
  I2C1FinishCode fc = i2c1_read_buff_sync(lis3mdl->address, reg, value, 1);
  return (lis3mdl_err_t)fc;
}
///////////////////////////////////////////////////////

lis3mdl_err_t _LIS3MDL_writeRegister(lis3mdl_t* lis3mdl,
                                      uint8_t reg,
                                      uint8_t data,
                                      uint8_t mask) {
  uint8_t valueToWrite, currentValue;
  _LIS3MDL_readRegister(lis3mdl, reg, &currentValue);
  valueToWrite = (currentValue & ~mask) | (data & mask);
  I2C1FinishCode fc = i2c1_write_buff_sync(lis3mdl->address, reg, &valueToWrite, 1);
  return (lis3mdl_err_t) fc;
}
///////////////////////////////////////////////////////

void LIS3MDL_read3AxisSync(lis3mdl_t *lis3mdl,
                           int16_t value[3]) {
  LIS3MDL_readAxis(lis3mdl, LIS3MDL_AXIS_X, &value[0]);
  LIS3MDL_readAxis(lis3mdl, LIS3MDL_AXIS_Y, &value[1]);
  LIS3MDL_readAxis(lis3mdl, LIS3MDL_AXIS_Z, &value[2]);  
}
///////////////////////////////////////////////////////

void LIS3MDL_enableODR(lis3mdl_t *lis3mdl, bool enable) {
  _LIS3MDL_writeRegister(lis3mdl, LIS3MDL_REG_CTL_1, enable, LIS3MDL_REG_CTL_1_ODR_EN);
}
///////////////////////////////////////////////////////

float LIS3MDL_currentGain(const lis3mdl_t *lis3mdl) {
  switch (lis3mdl->scale) {
    case 4: return 6842.0f;
    case 8: return 3421.0f;
    case 12: return 2281.0f;
    case 16: return 1711.0f;
  }
  return 1.0f;
}
