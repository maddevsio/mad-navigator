#ifndef __NEO6M
#define __NEO6M

#include <stdint.h>
#include <stdbool.h>

#include "stm32f10x.h"
#include "nmea.h"
#include "time/kov_time.h"

typedef struct neo6m_data {
  float latitude;
  float longitude;
  float altitude;
  kov_time_t time;
  kov_date_t date;
  float speed_knots;
  float course;
  float magnetic_variation; //!
} neo6m_data_t;

void neo6m_init(void);
void neo6m_power(bool on);
void neo6m_interrupts(bool enable);

extern volatile bool neo6m_needParseCurrentBuff;
neo6m_data_t neo6m_parseCurrentBuff(bool *valid);

#endif //__NEO6M
