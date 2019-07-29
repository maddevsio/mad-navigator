#ifndef QMC5883L_H
#define QMC5883L_H

#include <stdint.h>
#include <stdbool.h>
#include "i2c1/i2c1.h"

#define QMC5883L_I2C_ADDR (0x0d << 1)

#define QMC5883L_X_LSB 0x00
#define QMC5883L_X_MSB 0x01

#define QMC5883L_Y_LSB 0x02
#define QMC5883L_Y_MSB 0x03

#define QMC5883L_Z_LSB 0x04
#define QMC5883L_Z_MSB 0x05

#define QMC5883L_STATUS1 0x06
#define QMC5883L_STATUS1_DRDY (1 << 0)
#define QMC5883L_STATUS1_OVL (1 << 1)
#define QMC5883L_STATUS1_DOR (1 << 2)

#define QMC5883L_TEMP_LSB 0x07
#define QMC5883L_TEMP_MSB 0x08

#define QMC5883L_CTRL1 0x09
/*Two 8-bits registers are used to control the device configurations.
Control register 1 is located in address 09H, it sets the operational modes (MODE). output data update rate
(ODR), magnetic field measurement range or sensitivity of the sensors (RNG) and over sampling rate (OSR).
Control register 2 is located in address 0AH. It controls Interrupt Pin enabling (INT_ENB), Point roll over function
enabling(POL_PNT) and soft reset (SOFT_RST).*/
/*
Two bits of MODE registers can transfer mode of operations in the device, the two modes are Standby, and
Continuous measurements. The default mode after Power-on-Reset (POR) is standby. There is no any restriction
in the transferring between the modes.*/
#define QMC5883L_CTRL1_MODE_OFFSET 0
#define QMC5883L_CTRL1_MODE_STANDBY     0x00
#define QMC5883L_CTRL1_MODE_CONTINUOUS  0x01

/*
Output data rate is controlled by ODR registers. Four data update frequencies can be selected: 10Hz, 50Hz,
100Hz and 200Hz. For most of compassing applications, we recommend 10 Hz for low power consumption. For
gaming, the high update rate such as 100Hz or 200Hz can be used.*/
#define QMC5883L_CTRL1_ODR_OFFSET 2
#define QMC5883L_CTRL1_ODR_10   0x00
#define QMC5883L_CTRL1_ODR_50   0x01
#define QMC5883L_CTRL1_ODR_100  0x02
#define QMC5883L_CTRL1_ODR_200  0x03

/*Field ranges of the magnetic sensor can be selected through the register RNG. The full scale field range is
determined by the application environments. For magnetic clear environment, low field range such as +/- 2gauss
can be used. The field range goes hand in hand with the sensitivity of the magnetic sensor. The lowest field range
has the highest sensitivity, therefore, higher resolution.*/
#define QMC5883L_CTRL1_RNG_OFFSET 4
#define QMC5883L_CTRL1_RNG_2 0x00
#define QMC5883L_CTRL1_RNG_8 0x01

/*Over sample Rate (OSR) registers are used to control bandwidth of an internal digital filter. Larger OSR value
leads to smaller filter bandwidth, less in-band noise and higher power consumption. It could be used to reach a
good balance between noise and power. Four over sample ratio can be selected, 64, 128, 256 or 512.*/
#define QMC5883L_CTRL1_OSR_OFFSET 6
#define QMC5883L_CTRL1_OSR_512  0x00
#define QMC5883L_CTRL1_OSR_256  0x01
#define QMC5883L_CTRL1_OSR_128  0x02
#define QMC5883L_CTRL1_OSR_64   0x03


#define QMC5883L_CTRL2 0x09

/*Interrupt enabling is controlled by register INT_ENB in control register 2. Once the interrupt is enabled, it will flag
when new data is in Data Output Registers.
INT_ENB: “0”: enable interrupt PIN, “1”: disable interrupt PIN*/
#define QMC5883L_CTRL2_INT_ENB  0x00
#define QMC5883L_CTRL2_INT_DIS  0x01

/*Pointer roll-over function is controlled by ROL_PNT register. When the point roll-over function is enabled, the I2C
data pointer automatically rolls between 00H ~ 06H, if I2C read begins at any address among 00H~06H.
ROL_PNT: “0”: Normal, “1”: Enable pointer roll-over function*/
#define QMC5883L_CTRL2_ROL_PNT_EN  (1 << 6)

/*Soft Reset can be done by changing the register SOFT_RST to set. Soft reset can be invoked at any time of any
mode. For example, if soft reset occurs at the middle of continuous mode reading, QMC5883L immediately
switches to standby mode due to mode register is reset to “00” in default.
SOFT_RST: “0”: Normal“1”: Soft reset, restore default value of all registers.*/
#define QMC5883L_CTRL2_SOFT_RST (1 << 7)

#define QMC5883L_STATUS2 0x0c

void qmc5883l_init(void);
void qmc5883l_pin_config(void);
void qmc5883l_power(bool on);

typedef union {
  uint8_t raw8[6];
  uint16_t raw16[3];
  struct {
    int16_t mx, my, mz;
  } data;
} qmc5883l_raw_data_t;

void qmc5883lReadHeadingSync(qmc5883l_raw_data_t *data);
void qmc5883lReadHeadingAsync(qmc5883l_raw_data_t *data, volatile int8_t *finishCode, i2c1_pf_callback cb);
void qmc5883lFixRawData(qmc5883l_raw_data_t *data);

float qmc5883l_gain(void);

#endif // QMC5883L_H
