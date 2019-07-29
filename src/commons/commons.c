#include "commons/commons.h"
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

char *uint32ToStr(char *buff, uint32_t val, uint8_t buff_len){
  char* pbe = &buff[buff_len-1];
  for (*pbe = 0; val && (pbe >= buff);) {
    *(--pbe) = (val % 10) + '0';
    val /= 10;
  }
  return pbe;
}
//////////////////////////////////////////////////////////////////////////

char *int32ToStr(char *buff,
                 int32_t val,
                 uint8_t buff_len){
  int8_t neg = val < 0;
  char* pbe = &buff[buff_len-1];
  val *= neg ? -1 : 1;

  for (*pbe = 0; val && (pbe > buff);) {
    *(--pbe) = (val % 10) + '0';
    val /= 10;
  }

  if (neg && (pbe > buff))
    *(--pbe) = '-';
  return pbe;
}
//////////////////////////////////////////////////////////////////////////

void waitTicks(int ticks) {
  while (ticks--);
}
///////////////////////////////////////////////////////

#define MAX_PRECISION 10
char *ftoa(float f, char *buf, int precision) {
  static const float rounders[MAX_PRECISION + 1] = {
    0.5f,				// 0
    0.05f,				// 1
    0.005f,				// 2
    0.0005f,				// 3
    0.00005f,			// 4
    0.000005f,			// 5
    0.0000005f,			// 6
    0.00000005f,			// 7
    0.000000005f,		// 8
    0.0000000005f,		// 9
    0.00000000005f		// 10
  };

  char *ptr = buf;
  char *p = ptr;
  char *p1;
  char c;
  int32_t intPart;

  // check precision bounds
  if (precision > MAX_PRECISION)
    precision = MAX_PRECISION;

  // sign stuff
  if (f < 0) {
    f = -f;
    *ptr++ = '-';
  }

  if (precision < 0) {   // negative precision == automatic precision guess
    if (f < 1.0f) precision = 6;
    else if (f < 10.0f) precision = 5;
    else if (f < 100.0f) precision = 4;
    else if (f < 1000.0f) precision = 3;
    else if (f < 10000.0f) precision = 2;
    else if (f < 100000.0f) precision = 1;
    else precision = 0;
  }

  // round value according the precision
  if (precision)
    f += rounders[precision];

  // integer part...
  intPart = (int32_t)f;
  f -= intPart;

  if (!intPart)
    *ptr++ = '0';
  else {
    // save start pointer
    p = ptr;

    // convert (reverse order)
    while (intPart) {
      *p++ = '0' + intPart % 10;
      intPart /= 10;
    }
    // save end pos
    p1 = p;
    // reverse result
    while (p > ptr) {
      c = *--p;
      *p = *ptr;
      *ptr++ = c;
    }
    // restore end pos
    ptr = p1;
  }

  // decimal part
  if (precision) {
    // place decimal point
    *ptr++ = '.';
    // convert
    while (precision--) {
      f *= 10.0f;
      c = (char)f;
      *ptr++ = '0' + c;
      f -= c;
    }
  }
  // terminating zero
  *ptr = 0;
  return buf;
}
///////////////////////////////////////////////////////
