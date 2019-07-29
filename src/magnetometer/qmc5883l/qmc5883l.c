#include "magnetometer/qmc5883l/qmc5883l.h"

#define CURRENT_RNG QMC5883L_CTRL1_RNG_2

void qmc5883l_init() {
  uint8_t ctrl1 = (QMC5883L_CTRL1_MODE_CONTINUOUS << QMC5883L_CTRL1_MODE_OFFSET) |
                  (QMC5883L_CTRL1_ODR_100 << QMC5883L_CTRL1_ODR_OFFSET) |
                  (CURRENT_RNG << QMC5883L_CTRL1_RNG_OFFSET) |
                  (QMC5883L_CTRL1_OSR_128 << QMC5883L_CTRL1_OSR_OFFSET);
  uint8_t ctrl2 = QMC5883L_CTRL2_INT_DIS;

  i2c1_write_buff_sync(QMC5883L_I2C_ADDR, QMC5883L_CTRL1, &ctrl1, 1);
  i2c1_write_buff_sync(QMC5883L_I2C_ADDR, QMC5883L_CTRL2, &ctrl2, 1);
}
///////////////////////////////////////////////////////

#define QMC5883L_Port             GPIOB
#define QMC5883L_RCC_Port         RCC_APB2Periph_GPIOB
#define QMC5883L_VCC_PIN          GPIO_Pin_9

void qmc5883l_pin_config() {
  GPIO_InitTypeDef GPIO_cfg;
  RCC_APB2PeriphClockCmd(QMC5883L_RCC_Port, ENABLE);

  GPIO_cfg.GPIO_Pin = QMC5883L_VCC_PIN;
  GPIO_cfg.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_cfg.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(QMC5883L_Port, &GPIO_cfg);
}
///////////////////////////////////////////////////////

void qmc5883l_power(bool on) {
  void (*gpio_change)(GPIO_TypeDef*, uint16_t) = on ? GPIO_SetBits : GPIO_ResetBits;
  gpio_change(QMC5883L_Port, QMC5883L_VCC_PIN);
}
///////////////////////////////////////////////////////

void qmc5883lReadHeadingSync(qmc5883l_raw_data_t *data) {
  i2c1_read_buff_sync(QMC5883L_I2C_ADDR, QMC5883L_X_LSB, data->raw8, 6);
}
///////////////////////////////////////////////////////


void qmc5883lReadHeadingAsync(qmc5883l_raw_data_t *data,
                              volatile int8_t *finishCode,
                              i2c1_pf_callback cb) {
  i2c1_read_buff_async(QMC5883L_I2C_ADDR, QMC5883L_X_LSB, data->raw8, 6, finishCode, cb);
}
///////////////////////////////////////////////////////

float qmc5883l_gain() {
  switch (CURRENT_RNG) {
    case(QMC5883L_CTRL1_RNG_2):
      return 12000.0f;
    case(QMC5883L_CTRL1_RNG_8):
      return 3000.0f;
  }
  return 1.0f;
}
///////////////////////////////////////////////////////

void qmc5883lFixRawData(qmc5883l_raw_data_t *data) {
  //LSB then MSB. need to reorder. so:
  for (int i = 0; i < 3; ++i)
    data->raw16[i] = (int16_t) (((uint16_t) data->raw8[2*i + 1] << 8) | data->raw8[2*i]);
}
