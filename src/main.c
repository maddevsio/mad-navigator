/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "stm32f10x.h"
#include "stm32vldiscovery_utils.h"
#include "stm32f10x_rcc.h"

#include "accelerometer/mpu6050/mpu6050_i2c.h"
#include "gps/neo6m.h"
#include "display/display.h"
#include "time/kov_time.h"
#include "power/power.h"
#include "time/kov_time.h"
#include "flash/settings.h"

#include "commons/commons.h"
#include "commons/bearing.h"
#include "commons/madgwick.h"
#include "commons/magnetic.h"
#include "commons/quaternion.h"
#include "commons/vector3d.h"
#include "i2c1/i2c1.h"

#include "magnetometer/lis3mdl/lis3mdl_i2c.h"
#include "magnetometer/hmc5883/hmc5883.h"
#include "magnetometer/qmc5883l/qmc5883l.h"

static const vector3d_t m_vec_ix = {.x = 1.0f, .y = 0.0f, .z = 0.0f};
static const vector3d_t m_vec_iy = {.x = 0.0f, .y = 1.0f, .z = 0.0f};
static const vector3d_t m_vec_iz = {.x = 0.0f, .y = 0.0f, .z = 1.0f};

static AHRSData m_ahrs;
static mpu6050_raw_data_t m_imuRaw;
static qmc5883l_raw_data_t m_magRaw;

static quaternion_t m_dst_q;
static quaternion_t m_magnetic_decl_q;

static vector3d_t m_dst_vecX;
static vector3d_t m_dst_vecY;

static volatile float m_declination = 0.0f;
static volatile float m_bearing = 0.0f;

static volatile float m_currentAngleX = 0.0f;
static volatile float m_currentAngleY = 0.0f;

static volatile uint16_t m_seconds_from_start = 0;
static volatile bool m_need_update_time = false;

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

//!todo move to flash!
static const float m_dst_lat = 21.422513f;
static const float m_dst_lon = 39.826197f;
static const uint16_t m_work_time_sec = 60;
static uint32_t m_time_offset = 3600 * 6; //+6GMT

#define MagXMin 6010
#define MagXMax 6252
#define MagYMin 1175
#define MagYMax 4282
#define MagZMin 1895
#define MagZMax 6585

#define MagXBias ((MagXMin + MagXMax) / 2)
#define MagYBias ((MagYMin + MagYMax) / 2)
#define MagZBias ((MagZMin + MagZMax) / 2)

#define MagXAvgDelta ((MagXMax - MagXMin) / 2.0f)
#define MagYAvgDelta ((MagYMax - MagYMin) / 2.0f)
#define MagZAvgDelta ((MagZMax - MagZMin) / 2.0f)

#define MagAvgDelta ((MagXAvgDelta + MagYAvgDelta + MagZAvgDelta) / 3.0f)

#define MagXScale (MagAvgDelta / MagXAvgDelta)
#define MagYScale (MagAvgDelta / MagYAvgDelta)
#define MagZScale (MagAvgDelta / MagZAvgDelta)

#define imu_calibration_steps 3000
#define gyro_250_gain 131.0f
static int16_t m_gx_bias = -656;
static int16_t m_gy_bias = -368;
static int16_t m_gz_bias = 78;

static int16_t m_ax_bias = 0;
static int16_t m_ay_bias = 0;
static int16_t m_az_bias = 0;

static void gyrAccCalibrationFinishedMsg(void);

static void correctDstDirection(void);
static void updateCurrentAngle(void);
static void correctBearingAndDeclination(float alt, float lat, float lon, float year);
static void readAHRSRaw(void);
static void readAHRSData(void);
static void calculateOrientation(void);
static void readGPS(void);
static void initTIM2(void);
static void printCurrentDateTime(const kov_date_t *d, const kov_time_t *t);

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

void correctDstDirection(void) {
  m_dst_q = quaternion_new_vec(&m_vec_iz, m_bearing * deg2rad_coeff);
  m_dst_vecX = quaternion_transform_vec(&m_dst_q, &m_vec_ix);
  m_dst_vecY = quaternion_transform_vec(&m_dst_q, &m_vec_iy);
}
///////////////////////////////////////////////////////

void updateCurrentAngle(void) {
  quaternion_t q_curr = quaternion_new_wxyz(q0, q1, q2, q3);
  vector3d_t currVecX = quaternion_transform_vec(&q_curr, &m_vec_ix);
  vector3d_t currVecY = quaternion_transform_vec(&q_curr, &m_vec_iy);

  float angleX = vector_flatCos(&currVecX, &m_dst_vecX);
  float signX = vector_rotationSign(&currVecX, &m_dst_vecX);
  angleX = acosf(angleX);
  m_currentAngleX = signX * rad2deg_coeff * angleX;

  float angleY = vector_flatCos(&currVecY, &m_dst_vecY);
  float signY = vector_rotationSign(&currVecY, &m_dst_vecY);
  angleY = acosf(angleY);
  m_currentAngleY = signY * rad2deg_coeff * angleY;
}
///////////////////////////////////////////////////////

void correctBearingAndDeclination(float alt, float lat,
                                  float lon, float year) {
  float declination = 0.0f;
  magnetic_calculate(alt / 1000.f, lat, lon,
                     year, &declination,
                     NULL, NULL, NULL);
  m_declination = declination;
  m_bearing = initialBearing(lat, lon, m_dst_lat, m_dst_lon);
  m_bearing = 360.0f - fmodf(m_bearing + 360.0f, 360.0f);
  m_magnetic_decl_q = quaternion_new_vec(&m_vec_iz, m_declination * deg2rad_coeff);
}
///////////////////////////////////////////////////////

void readAHRSRaw(void) {
  mpu6050ReadSync(&m_imuRaw);
  mpu6050FixData(&m_imuRaw);

  qmc5883lReadHeadingSync(&m_magRaw);
  qmc5883lFixRawData(&m_magRaw);
}
///////////////////////////////////////////////////////

void readAHRSData(void) {
  float currentMagGain = qmc5883l_gain();
  readAHRSRaw();
  //we can use raw data from accelerometer because
  //it's normalized inside Madgwick filter
  m_ahrs.imu.acc.ax = (float)m_imuRaw.data.ax;
  m_ahrs.imu.acc.ay = (float)m_imuRaw.data.ay;
  m_ahrs.imu.acc.az = (float)m_imuRaw.data.az;

  //but we have to use radian/sec as unit for gyroscope
  m_imuRaw.data.gx -= m_gx_bias;
  m_imuRaw.data.gy -= m_gy_bias;
  m_imuRaw.data.gz -= m_gz_bias;

  m_ahrs.imu.gyro.gx = (float)(m_imuRaw.data.gx / gyro_250_gain) * deg2rad_coeff;
  m_ahrs.imu.gyro.gy = (float)(m_imuRaw.data.gy / gyro_250_gain) * deg2rad_coeff;
  m_ahrs.imu.gyro.gz = (float)(m_imuRaw.data.gz / gyro_250_gain) * deg2rad_coeff;

  //remove hard iron distortion + use ellipse rescaling.
  //https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
  //!todo soft iron distortion compensation with matrix
  m_ahrs.mag.mx = (float)(m_magRaw.data.mx - MagXBias) * MagXScale;
  m_ahrs.mag.my = (float)(m_magRaw.data.my - MagYBias) * MagYScale;
  m_ahrs.mag.mz = (float)(m_magRaw.data.mz - MagZBias) * MagZScale;
  m_ahrs.mag.mx /= currentMagGain;
  m_ahrs.mag.my /= currentMagGain;
  m_ahrs.mag.mz /= currentMagGain;

  //to microgauss
  m_ahrs.mag.mx *= 1000000.f;
  m_ahrs.mag.my *= 1000000.f;
  m_ahrs.mag.mz *= 1000000.f;
}
///////////////////////////////////////////////////////

void calculateOrientation(void) {
  vector3d_t magv = vector_new(m_ahrs.mag.mx, m_ahrs.mag.my, m_ahrs.mag.mz);
  vector3d_t magvc = quaternion_transform_vec(&m_magnetic_decl_q, &magv);
  readAHRSData();
  //be careful here. see  directions of magnetometer and acclerometer with gyroscope
  MadgwickAHRSupdate(m_ahrs.imu.gyro.gx, m_ahrs.imu.gyro.gy, m_ahrs.imu.gyro.gz,
                     m_ahrs.imu.acc.ax, m_ahrs.imu.acc.ay, m_ahrs.imu.acc.az,
                     magvc.x, magvc.y, magvc.z);
  updateCurrentAngle();

  static char buff[16] = {0};
  quaternion_t q = quaternion_new_wxyz(q0, q1, q2, q3);
  display->printText(0, 0, "x:");
  display->printText(2, 0, ftoa(m_currentAngleX, buff, 5));
  display->printText(0, 1, "y:");
  display->printText(2, 1, ftoa(quaternion_yaw(&q) * rad2deg_coeff, buff, 5));
}
///////////////////////////////////////////////////////

#define TS_CORRECTION_THRESHOLD_SEC 5
void readGPS(void) {
  if (!neo6m_needParseCurrentBuff)
    return;
  neo6m_needParseCurrentBuff = false;
  bool valid = false;
  neo6m_data_t d = neo6m_parseCurrentBuff(&valid);
  if (!valid)
    return;

  neo6m_interrupts(false);
  kov_timestamp_t ts_gps = nmea_gettime(&d.date, &d.time);
  kov_timestamp_t ts_loc = RTC_CurrentTimestamp();
  ts_loc -= m_time_offset;

  uint32_t ts_diff = ts_gps >= ts_loc ? ts_gps - ts_loc : ts_loc - ts_gps;
  if (ts_diff > TS_CORRECTION_THRESHOLD_SEC)
    RTC_Adjust(ts_gps + m_time_offset);

  float year = d.date.year + (float)d.date.month / 12.0f;

  /*todo get current altitude from GPS*/
  correctBearingAndDeclination(0.0f, d.latitude, d.longitude, year);
  correctDstDirection();
  neo6m_power(false); //save energy, turn GPS OFF! :)
}
///////////////////////////////////////////////////////

void initTIM2(void) {
  /*configure tim2*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef tim2_cfg;

  tim2_cfg.TIM_ClockDivision = TIM_CKD_DIV1; //24 000 000
  tim2_cfg.TIM_Prescaler = 23999; //1KHz
  tim2_cfg.TIM_CounterMode = TIM_CounterMode_Up;
  tim2_cfg.TIM_Period = 999; // 1.0 second!
  tim2_cfg.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM2, &tim2_cfg);
  TIM_Cmd(TIM2, ENABLE);

  /*configure NVIC*/
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 1;
  nvicStructure.NVIC_IRQChannelSubPriority = 0;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}
///////////////////////////////////////////////////////

void TIM2_IRQHandler() {
  if (TIM2->SR & TIM_IT_Update) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    ++m_seconds_from_start;
    m_need_update_time = true;
  }
}
///////////////////////////////////////////////////////

void printCurrentDateTime(const kov_date_t *d, const kov_time_t *t) {
  display->printText(0, 0, RTC_DateTimeStr(d, t));
}
///////////////////////////////////////////////////////

typedef enum operation_mode {
  OP_NORMAL = 0,
  OP_ACC_CALIBRATION = 1,
  OP_GYR_CALIBRATION = 2,
  OP_MAG_CALIBRATION = 3
} operation_mode_t;

static operation_mode_t m_mode = OP_GYR_CALIBRATION;

static void op_normal(void);
static void op_acc_calibration(void);
static void op_gyr_calibration(void);
static void op_mag_calibration(void);
static void magnetometerCalibration(int16_t mx, int16_t my, int16_t mz);
static void gyroscopeCalibration(int16_t gx, int16_t gy, int16_t gz);
static void accelerometerCalibration(int16_t ax, int16_t ay, int16_t az);

void gyrAccCalibrationFinishedMsg(void) {
  display->clear();
  display->printText(0, 0, "Calibration");
  display->printText(0, 1, "is finished");
  display->printText(0, 2, "Wait!");
  display->syncAll();
  waitTicks(12000000); //~0.5 sec
  display->clear();
}
///////////////////////////////////////////////////////

void magnetometerCalibration(int16_t mx, int16_t my, int16_t mz) {
  static char buff[32] = {0};
  static volatile int16_t m_magX_min = INT16_MAX, m_magX_max = INT16_MIN;
  static volatile int16_t m_magY_min = INT16_MAX, m_magY_max = INT16_MIN;
  static volatile int16_t m_magZ_min = INT16_MAX, m_magZ_max = INT16_MIN;

  if (mx < m_magX_min)
    m_magX_min = mx;
  if (mx > m_magX_max)
    m_magX_max = mx;

  if (my < m_magY_min)
    m_magY_min = my;
  if (my > m_magY_max)
    m_magY_max = my;

  if (mz < m_magZ_min)
    m_magZ_min = mz;
  if (mz > m_magZ_max)
    m_magZ_max = mz;

  display->printText(0, 0, "x0:");
  display->printText(3, 0, int32ToStr(buff, m_magX_min, sizeof(buff)));
  display->printText(0, 1, "x1:");
  display->printText(3, 1, int32ToStr(buff, m_magX_max, sizeof(buff)));
  display->printText(0, 2, "y0:");
  display->printText(3, 2, int32ToStr(buff, m_magY_min, sizeof(buff)));
  display->printText(0, 3, "y1:");
  display->printText(3, 3, int32ToStr(buff, m_magY_max, sizeof(buff)));
  display->printText(0, 4, "z0:");
  display->printText(3, 4, int32ToStr(buff, m_magZ_min, sizeof(buff)));
  display->printText(0, 5, "z1:");
  display->printText(3, 5, int32ToStr(buff, m_magZ_max, sizeof(buff)));
  display->syncAll();
}
///////////////////////////////////////////////////////

void gyroscopeCalibration(int16_t gx, int16_t gy, int16_t gz) {
  static float gx_mean = 0.0f, gy_mean = 0.0f, gz_mean = 0.0f;
  static uint16_t count = imu_calibration_steps;
  if (count == 0) {
    gx_mean /= (float)imu_calibration_steps;
    gy_mean /= (float)imu_calibration_steps;
    gz_mean /= (float)imu_calibration_steps;
    m_gx_bias = (int16_t) (ceilf(gx_mean));
    m_gy_bias = (int16_t) (ceilf(gy_mean));
    m_gz_bias = (int16_t) (ceilf(gz_mean));
    gyrAccCalibrationFinishedMsg();
    m_mode = OP_NORMAL;
  }
  gx_mean += (float)gx;
  gy_mean += (float)gy;
  gz_mean += (float)gz;
  --count;
}
///////////////////////////////////////////////////////

void accelerometerCalibration(int16_t ax, int16_t ay, int16_t az) {
  static float ax_mean = 0.0f, ay_mean = 0.0f, az_mean = 0.0f;
  static uint16_t count = imu_calibration_steps;
  if (count == 0) {
    ax_mean /= (float)imu_calibration_steps;
    ay_mean /= (float)imu_calibration_steps;
    az_mean /= (float)imu_calibration_steps;
    m_ax_bias = (int16_t) (ceilf(ax_mean));
    m_ay_bias = (int16_t) (ceilf(ay_mean));
    m_az_bias = (int16_t) (ceilf(az_mean));
    gyrAccCalibrationFinishedMsg();
    m_mode = OP_NORMAL;
  }
  ax_mean += (float)ax;
  ay_mean += (float)ay;
  az_mean += (float)az;
  --count;
}
///////////////////////////////////////////////////////

void op_normal(void) {
  static float deltaAngleX = 0.0f;
  static float prevAngleX = 0.0f;
#define deltaAngleThreshold 1.5f

  if (m_need_update_time) {
    kov_date_t d;
    kov_time_t t;
    kov_timestamp_t ts = RTC_CurrentTimestamp();
    RTC_GetDateTime(ts, &d, &t);
    printCurrentDateTime(&d, &t);
    m_need_update_time = false;
  }

  readGPS();
  calculateOrientation();

  deltaAngleX = m_currentAngleX - prevAngleX;
  if (deltaAngleX > deltaAngleThreshold || deltaAngleX < -deltaAngleThreshold) { //abs(dx) < da
    drawCircleFromBuffer(42, 27, true);
    drawArrow(42, 27, deg2rad_coeff * (-m_currentAngleX), false);
    prevAngleX = m_currentAngleX;
  }
  display->syncAll();
  if(m_seconds_from_start > m_work_time_sec) {
//    goToSleepMode();
  }
}
///////////////////////////////////////////////////////

void op_acc_calibration(void) {
  mpu6050ReadSync(&m_imuRaw);
  mpu6050FixData(&m_imuRaw);
  accelerometerCalibration(m_imuRaw.data.ax, m_imuRaw.data.ay, m_imuRaw.data.az);
}
///////////////////////////////////////////////////////

void op_gyr_calibration(void) {
  mpu6050ReadSync(&m_imuRaw);
  mpu6050FixData(&m_imuRaw);
  gyroscopeCalibration(m_imuRaw.data.gx, m_imuRaw.data.gy, m_imuRaw.data.gz);
}
///////////////////////////////////////////////////////

void op_mag_calibration(void) {
  qmc5883lReadHeadingSync(&m_magRaw);
  qmc5883lFixRawData(&m_magRaw);
  magnetometerCalibration(m_magRaw.data.mx, m_magRaw.data.my, m_magRaw.data.mz);
}
///////////////////////////////////////////////////////

static uint32_t m_pvd_current = PWR_PVDLevel_2V9 + 0x20;
static void pvd_ctrl(void) {
  if (PWR->CSR & PWR_CSR_PVDO) {
    if (m_pvd_current == PWR_PVDLevel_2V2) {
      //!todo some message that need to charge
      return;
    }

    m_pvd_current -= 0x20; //WARNING!
    PWR_PVDLevelConfig(m_pvd_current);
    return;
  }
}
///////////////////////////////////////////////////////

static void pvd_print(void) {
  float percent = (float)m_pvd_current / (float)(PWR_PVDLevel_2V9 + 0x20);

  /*print empty rectangle*/
#define left 76
#define right 83
#define top 10
#define bottom 47
#define full_height (bottom - top)

  drawLine(left, top, left, bottom, true); //left
  drawLine(left, top, right, top, true); //top
  drawLine(right, top, right, bottom, true); //right
  drawLine(left, bottom, right, bottom, true);

  /*print full rectangle by percentage*/
  int16_t ch = (int16_t)(percent * full_height);
  for (int16_t x = left+1; x < right; ++x) {
    drawLine(x, bottom, x, bottom - ch, true);
  }
  display->syncAll();
}
///////////////////////////////////////////////////////

int main(void) {
  /* Enable PWR and BKP clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  PWR_PVDLevelConfig(m_pvd_current);
  PWR_PVDCmd(ENABLE);

  MadgwickInit(130.0f, 0.4f);
//  initDWT();

  //todo make some indication in case of some of modules doesn't work
  //if display doesn't work - blink with diod
  //if any another module doesn't work - message on display and exit.
  display = initDisplay();
  display->setBacklight(true);
  display->clear();

  i2c1_init();
  display->printText(0, 0, "I2C1 init!");
  display->syncAll();

  mpu6050_pin_config();
  mpu6050_power(true);
  waitTicks(50000);
  mpu6050_init();
  display->printText(0, 1, "MPU6050 init!");
  display->syncAll();

  qmc5883l_pin_config();
  qmc5883l_power(true);
  waitTicks(50000);
  qmc5883l_init();
  display->printText(0, 2, "QMC5883 init!");
  display->syncAll();

  neo6m_init();
  neo6m_power(true);
  waitTicks(50000);
  neo6m_interrupts(true);
  display->printText(0, 3, "USART3 init!");
  display->syncAll();

  RTC_Init();
  display->printText(0, 4, "RTC init!");
  display->syncAll();

  initWakeup();
  display->printText(0, 5, "WAKEUP init!");
  display->syncAll();

  //!todo remove after tests , read from FLASH!
#define bish_lat 42.879965f
#define bish_lon 74.617977f

#define mos_lat 55.752124f
#define mos_lon 37.619779f

  correctBearingAndDeclination(0.0f, bish_lat, bish_lon, 2019.5);
  correctDstDirection();
  //!!!!!!!!!!!!!!!!!!!!

  initTIM2();

  display->syncAll();
  display->clear();
  calcCircleBuffer(21);

  pvd_ctrl(); //todo move
  pvd_print();

  while (1) {
    switch (m_mode) {
      case OP_NORMAL:
        op_normal();
        break;
      case OP_ACC_CALIBRATION:
        op_acc_calibration();
        break;
      case OP_GYR_CALIBRATION:
        op_gyr_calibration();
        break;
      case OP_MAG_CALIBRATION:
        op_mag_calibration();
        break;
    }
  }
//  return 0;
}
