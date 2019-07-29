#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "time/kov_time.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_rcc.h"
#include "display/display.h"
#include "commons/commons.h"

#define BACKUP_MAGIC 0xAD74


static volatile bool isInUpdate = false;
static void RTC_config(void);

kov_timestamp_t RTC_CurrentTimestamp() {
  return RTC_GetCounter();
}
///////////////////////////////////////////////////////

bool RTC_Adjust(kov_timestamp_t ts) {  
  if(!isInUpdate) {
    isInUpdate = true; //do we have some atomics?
    PWR_BackupAccessCmd(ENABLE);
    RTC_WaitForLastTask();
    RTC_SetCounter(ts);
    RTC_WaitForLastTask();
    PWR_BackupAccessCmd(DISABLE);
    isInUpdate = false;
  }
  return false;
}
///////////////////////////////////////////////////////

void RTC_Init(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  if (BKP_ReadBackupRegister(BKP_DR1) != BACKUP_MAGIC) {
    RTC_config();
    BKP_WriteBackupRegister(BKP_DR1, BACKUP_MAGIC);
  } else {
    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
  }
  RCC_ClearFlag();
}
///////////////////////////////////////////////////////

void RTC_config(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  PWR_BackupAccessCmd(ENABLE);
  // Если RTC выключен - инициализировать
  if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN) {
    // Сброс данных в резервной области
    RCC_BackupResetCmd(ENABLE);
    RCC_BackupResetCmd(DISABLE);
    // Установить источник тактирования кварц 32768
    RCC_LSEConfig(RCC_LSE_ON);
    while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY) {}
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    RTC_SetPrescaler(0x7FFF); // Устанавливаем делитель 32767, чтобы часы считали секунды
    // Включаем RTC
    RCC_RTCCLKCmd(ENABLE);
    // Ждем синхронизацию
    RTC_WaitForSynchro();
  }
}

///////////////////////////////////////////////////////

// Get current date
//https://ru.m.wikipedia.org/wiki/%D0%AE%D0%BB%D0%B8%D0%B0%D0%BD%D1%81%D0%BA%D0%B0%D1%8F_%D0%B4%D0%B0%D1%82%D0%B0
void RTC_GetDateTime(kov_timestamp_t ts,
                     kov_date_t *date,
                     kov_time_t *time) {
  uint32_t ct;
  uint32_t t1, a, b, c, d, e, m;
  uint32_t jd = 0;
  uint32_t jdn = 0;

  uint16_t year = 0;
  uint8_t mon = 0;
  uint16_t mday = 0;
  uint32_t hour = 0;
  uint32_t min = 0;
  uint32_t sec = 0;

  jd = ((ts+43200)/(86400>>1)) + (2440587<<1) + 1;
  jdn = jd>>1;

  ct = ts;
  t1 = ct / 60u;
  sec = ct - t1*60u;

  ct = t1;
  t1 = ct/60;
  min = ct - t1*60;

  ct = t1;
  t1 = ct/24;
  hour = ct - t1*24;

  a = jdn + 32044;
  b = (4*a + 3) / 146097;
  c = a - (146097*b) / 4;
  d = (4*c + 3) / 1461;
  e = c - (1461*d) / 4;
  m = (5*e + 2) / 153;
  mday = e - (153*m + 2) / 5 + 1;
  mon = m + 3 - 12 * (m / 10);
  year = 100*b + d - 4800 + (m / 10);

  date->year = year;
  date->month = mon;
  date->day = mday;
  time->hours = hour;
  time->minutes = min;
  time->seconds = sec;
}

#define JULIAN_DATE_BASE    2440588
// Convert Date to Counter
kov_timestamp_t RTC_GetTimestampDateTime(const kov_date_t *date,
                                 const kov_time_t *time) {
  return RTC_GetTimestamp(date->year, date->month, date->day,
                          time->hours, time->minutes, time->seconds);
}
///////////////////////////////////////////////////////

kov_timestamp_t RTC_GetTimestamp(int32_t year,
                                 int32_t month,
                                 int32_t day,
                                 int32_t hours,
                                 int32_t minutes,
                                 int32_t seconds) {
  uint8_t a;
  uint16_t y;
  uint8_t m;
  kov_timestamp_t JDN;

  a = (14 - month) / 12;
  y = year + 4800 - a;
  m = month + (12 * a) - 3;

  JDN = day;
  JDN += (153*m + 2) / 5;
  JDN += 365 * y;
  JDN += y / 4;
  JDN += -y / 100;
  JDN += y / 400;
  JDN = JDN - 32045;
  JDN = JDN - JULIAN_DATE_BASE;
  JDN *= 86400;

  JDN += hours * 3600;
  JDN += minutes * 60;
  JDN += seconds;

  return JDN;
}
///////////////////////////////////////////////////////

const char *RTC_TimeStr(const kov_time_t *t) {
  static char buff[9] = {0};
  if (t->hours < 10)
    buff[0] = '0';
  if (t->hours)
    uint32ToStr(buff, t->hours, 3);
  else
    buff[1] = '0';

  buff[2] = ':';
  if (t->minutes < 10)
    buff[3] = '0';
  if (t->minutes)
    uint32ToStr(&buff[3], t->minutes, 3);
  else
    buff[4] = '0';

  buff[5] = ':';
  if (t->seconds < 10)
    buff[6] = '0';
  if (t->seconds)
    uint32ToStr(&buff[6], t->seconds, 3);
  else
    buff[7] = '0';
  return buff;
}
///////////////////////////////////////////////////////

static const char *months[] = {"jan", "feb", "mar", "apr", "may", "jun", "jul", "aug", "sep", "oct", "nov", "dec"};
const char *RTC_DateStr(const kov_date_t *d) {
  static char buff[12] = {0};
  if (d->day < 10)
    buff[0] = '0';
  uint32ToStr(buff, d->day, 3);
  buff[2] = ' ';
  memcpy((void*)&buff[3], months[d->month - 1], 3);
  buff[6] = ' ';
  uint32ToStr(&buff[7], d->year, 5);
  return buff;
}
///////////////////////////////////////////////////////

const char *RTC_DateTimeStr(const kov_date_t *d,
                            const kov_time_t *t) {
  static char buff[13] = {0};
  if (d->day < 10)
    buff[0] = '0';
  uint32ToStr(buff, d->day, 3);
  buff[2] = ' ';
  memcpy((void*)&buff[3], months[d->month - 1], 3);
  buff[6] = ' ';

  if (t->hours < 10)
    buff[7] = '0';
  if (t->hours)
    uint32ToStr(&buff[7], t->hours, 3);
  else
    buff[8] = '0';

  buff[9] = ':';
  if (t->minutes < 10)
    buff[10] = '0';
  if (t->minutes)
    uint32ToStr(&buff[10], t->minutes, 3);
  else
    buff[11] = '0';

  return buff;
}
///////////////////////////////////////////////////////
