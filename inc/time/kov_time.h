#ifndef __KOV_TIME_H
#define __KOV_TIME_H

#include <stdint.h>
#include <stdbool.h>

typedef struct kov_date {
  uint16_t day;
  uint8_t month;
  uint16_t year;
} kov_date_t;

typedef struct kov_time {
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint8_t microseconds;
} kov_time_t;

typedef uint32_t kov_timestamp_t;

void RTC_Init(void);
kov_timestamp_t RTC_CurrentTimestamp(void);
bool RTC_Adjust(kov_timestamp_t ts);

//!WARNING!! These functions will work until 2106 February 07. Then need to change to 64 bit! or apply some offset :)
void RTC_GetDateTime(kov_timestamp_t ts, kov_date_t *date, kov_time_t *time);
kov_timestamp_t RTC_GetTimestampDateTime(const kov_date_t *date,const kov_time_t *time);
kov_timestamp_t RTC_GetTimestamp(int32_t year, int32_t month, int32_t day,
                                 int32_t hours, int32_t minutes, int32_t seconds);

const char *RTC_TimeStr(const kov_time_t *t);
const char *RTC_DateStr(const kov_date_t *d);
const char *RTC_DateTimeStr(const kov_date_t *d, const kov_time_t *t);

#endif
