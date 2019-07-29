TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += STM32 \
    STM32F1 \
    STM32F100RBTx \
    DEBUG \
    STM32F10X_MD_VL \
    USE_STDPERIPH_DRIVER \
    BEARING_HIGH_PRECISION

INCLUDEPATH += inc \
  CMSIS/core \
  CMSIS/device \
  StdPeriph_Driver/inc \
  inc/gps \
  inc/mpu6050 \
  inc/nokiaLcd

SOURCES += \
    src/commons/bearing.c \
    src/commons/commons.c \
    src/commons/madgwick.c \
    src/commons/magnetic.c \
    src/commons/quaternion.c \
    src/commons/vector3d.c \
    src/display/display.c \
    src/display/nokiaLcd/nokia5110.c \
    src/display/nokiaLcd/nokia5110_buffered.c \
    src/display/nokiaLcd/nokia5110_font.c \
    src/gps/nmea.c \
    src/magnetometer/hmc5883/hmc5883.c \
    src/i2c1/i2c1.c \
    src/magnetometer/lis3mdl/lis3mdl_i2c.c \
    src/magnetometer/qmc5883l/qmc5883l.c \
    src/main.c \
    src/stm32vldiscovery_utils.c \
    src/syscalls.c \
    src/system_stm32f10x.c \
    src/gps/neo6m.c \
    src/accelerometer/mpu6050/MPU6050.c \
    src/accelerometer/mpu6050/mpu6050_i2c.c \
    src/nokiaLcd/stm32f10x_pcd8544.c \
    src/power/power.c \
    CMSIS/core/core_cm3.c \
    StdPeriph_Driver/src/misc.c \
    StdPeriph_Driver/src/stm32f10x_adc.c \
    StdPeriph_Driver/src/stm32f10x_bkp.c \
    StdPeriph_Driver/src/stm32f10x_can.c \
    StdPeriph_Driver/src/stm32f10x_cec.c \
    StdPeriph_Driver/src/stm32f10x_crc.c \
    StdPeriph_Driver/src/stm32f10x_dac.c \
    StdPeriph_Driver/src/stm32f10x_dbgmcu.c \
    StdPeriph_Driver/src/stm32f10x_dma.c \
    StdPeriph_Driver/src/stm32f10x_exti.c \
    StdPeriph_Driver/src/stm32f10x_flash.c \
    StdPeriph_Driver/src/stm32f10x_fsmc.c \
    StdPeriph_Driver/src/stm32f10x_gpio.c \
    StdPeriph_Driver/src/stm32f10x_i2c.c \
    StdPeriph_Driver/src/stm32f10x_iwdg.c \
    StdPeriph_Driver/src/stm32f10x_pwr.c \
    StdPeriph_Driver/src/stm32f10x_rcc.c \
    StdPeriph_Driver/src/stm32f10x_rtc.c \
    StdPeriph_Driver/src/stm32f10x_sdio.c \
    StdPeriph_Driver/src/stm32f10x_spi.c \
    StdPeriph_Driver/src/stm32f10x_tim.c \
    StdPeriph_Driver/src/stm32f10x_usart.c \
    StdPeriph_Driver/src/stm32f10x_wwdg.c \
    src/time/kov_time.c

DISTFILES += \
    Makefile \
    startup/startup_stm32.s \
    LinkerScript.ld

HEADERS += \
    CMSIS/core/core_cm3.h \
    CMSIS/device/stm32f10x.h \
    CMSIS/device/system_stm32f10x.h \
    inc/commons/bearing.h \
    inc/commons/commons.h \
    inc/commons/madgwick.h \
    inc/commons/magnetic.h \
    inc/commons/quaternion.h \
    inc/commons/vector3d.h \
    inc/display/display.h \
    inc/display/nokiaLcd/nokia5110.h \
    inc/display/nokiaLcd/nokia5110_buffered.h \
    inc/display/nokiaLcd/nokia5110_font.h \
    inc/gps/nmea.h \
    inc/magnetometer/hmc5883/hmc5883.h \
    inc/i2c1/i2c1.h \
    inc/magnetometer/lis3mdl/lis3mdl_i2c.h \
    inc/magnetometer/qmc5883l/qmc5883l.h \
    inc/stm32f10x_it.h \
    inc/gps/neo6m.h \
    inc/accelerometer/mpu6050/MPU6050.h \
    inc/accelerometer/mpu6050/mpu6050_i2c.h \
    inc/nokiaLcd/stm32f10x_pcd8544.h \
    inc/stm32vldiscovery_utils.h \
    StdPeriph_Driver/inc/misc.h \
    StdPeriph_Driver/inc/stm32f10x_adc.h \
    StdPeriph_Driver/inc/stm32f10x_bkp.h \
    StdPeriph_Driver/inc/stm32f10x_can.h \
    StdPeriph_Driver/inc/stm32f10x_cec.h \
    StdPeriph_Driver/inc/stm32f10x_conf.h \
    StdPeriph_Driver/inc/stm32f10x_crc.h \
    StdPeriph_Driver/inc/stm32f10x_dac.h \
    StdPeriph_Driver/inc/stm32f10x_dbgmcu.h \
    StdPeriph_Driver/inc/stm32f10x_dma.h \
    StdPeriph_Driver/inc/stm32f10x_exti.h \
    StdPeriph_Driver/inc/stm32f10x_flash.h \
    StdPeriph_Driver/inc/stm32f10x_fsmc.h \
    StdPeriph_Driver/inc/stm32f10x_gpio.h \
    StdPeriph_Driver/inc/stm32f10x_i2c.h \
    StdPeriph_Driver/inc/stm32f10x_iwdg.h \
    StdPeriph_Driver/inc/stm32f10x_pwr.h \
    StdPeriph_Driver/inc/stm32f10x_rcc.h \
    StdPeriph_Driver/inc/stm32f10x_rtc.h \
    StdPeriph_Driver/inc/stm32f10x_sdio.h \
    StdPeriph_Driver/inc/stm32f10x_spi.h \
    StdPeriph_Driver/inc/stm32f10x_tim.h \
    StdPeriph_Driver/inc/stm32f10x_usart.h \
    StdPeriph_Driver/inc/stm32f10x_wwdg.h \
    inc/time/kov_time.h \
    inc/hardware.h

