#include "stm32f10x_pwr.h"
#include "display/display.h"
#include "gps/neo6m.h"
#include "accelerometer/mpu6050/mpu6050_i2c.h"
#include "magnetometer/qmc5883l/qmc5883l.h"

void goToSleepMode(void) {  
  display->clear();
  display->syncAll();
  display->setBacklight(0);

  mpu6050_power(false);
  neo6m_power(false);
  qmc5883l_power(false);

  PWR_EnterSTANDBYMode();
}

void initWakeup(void) {
  PWR_WakeUpPinCmd(ENABLE);
}
