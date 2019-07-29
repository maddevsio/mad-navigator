#ifndef I2C1_H
#define I2C1_H

#include <stdint.h>
#include <stm32f10x.h>

typedef enum {
  FC_FINISHED = 0,
  FC_IN_PROGRESS  = 1,
  FC_I2C_ERR = 2,
  FC_DFA_ERR = 3
} I2C1FinishCode;

void i2c1_init(void);
void i2c1_interrupts(FunctionalState on);

typedef void (*i2c1_pf_callback)(I2C1FinishCode);

void i2c1_read_buff_async(uint8_t slaveAddr, uint8_t startReg, uint8_t *buff, uint8_t len, volatile int8_t *finishCode, i2c1_pf_callback cb);
I2C1FinishCode i2c1_read_buff_sync(uint8_t slaveAddr, uint8_t startReg, uint8_t *buff, uint8_t len);

void i2c1_write_buff_async(uint8_t slaveAddr, uint8_t startReg, uint8_t *buff, uint8_t len, volatile int8_t *finishCode, i2c1_pf_callback cb);
I2C1FinishCode i2c1_write_buff_sync(uint8_t slaveAddr, uint8_t startReg, uint8_t *buff, uint8_t len);

void i2c1_scan(void (*cb)(uint8_t, uint8_t));

#endif // I2C1_H
