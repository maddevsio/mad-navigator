#include <stddef.h>
#include "accelerometer/mpu6050/mpu6050_i2c.h"
#include "commons/commons.h"
#include "i2c1/i2c1.h"

#define MPU6050_Port             GPIOB
#define MPU6050_RCC_Port         RCC_APB2Periph_GPIOB
#define MPU6050_VCC_PIN          GPIO_Pin_8

void mpu6050_pin_config() {
  GPIO_InitTypeDef GPIO_cfg;
  RCC_APB2PeriphClockCmd(MPU6050_RCC_Port, ENABLE);

  GPIO_cfg.GPIO_Pin = MPU6050_VCC_PIN;
  GPIO_cfg.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_cfg.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(MPU6050_Port, &GPIO_cfg);
}

void mpu6050_power(bool on) {
  void (*gpio_change)(GPIO_TypeDef*, uint16_t) = on ? GPIO_SetBits : GPIO_ResetBits;
  gpio_change(MPU6050_Port, MPU6050_VCC_PIN);
}
///////////////////////////////////////////////////////

void mpu6050_init(void) {  
  MPU6050_Initialize();    
}
///////////////////////////////////////////////////////

void mpu6050ReadAsync(mpu6050_raw_data_t *data, volatile int8_t *finishCode, i2c1_pf_callback cb) {
  i2c1_read_buff_async(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, data->raw8, 14, finishCode, cb);
}
///////////////////////////////////////////////////////

void mpu6050ReadSync(mpu6050_raw_data_t *data) {
  i2c1_read_buff_sync(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, data->raw8, 14);
}
///////////////////////////////////////////////////////

void mpu6050FixData(mpu6050_raw_data_t *imuRaw) {
  for (int i = 0; i < 7; ++i)
    imuRaw->raw16[i] = (int16_t) ((uint16_t) imuRaw->raw8[2 * i] << 8) | imuRaw->raw8[2 * i + 1];
}
///////////////////////////////////////////////////////
