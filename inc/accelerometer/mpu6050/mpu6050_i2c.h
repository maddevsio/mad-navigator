#ifndef MPU6050_I2C_H
#define MPU6050_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "MPU6050.h"
#include "i2c1/i2c1.h"

typedef union {
  uint8_t raw8[14];
  int16_t raw16[7];
  struct {
    int16_t ax, ay, az;
    int16_t t;
    int16_t gx, gy, gz;
  } data;
} mpu6050_raw_data_t;

void mpu6050_init(void);
void mpu6050_pin_config(void);
void mpu6050_power(bool on);

void mpu6050ReadSync(mpu6050_raw_data_t *data);
void mpu6050ReadAsync(mpu6050_raw_data_t *data, volatile int8_t *finishCode, i2c1_pf_callback cb);
void mpu6050FixData(mpu6050_raw_data_t *imuRaw);

#endif //MPU6050_I2C_H
