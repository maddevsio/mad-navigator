#ifndef __STM32F10X_PCD8544_BUFFERED_H_
#define __STM32F10X_PCD8544_BUFFERED_H_

#include <stdint.h>
#include <stdbool.h>
#include "nokia5110_font.h"

void nokia5110_bufferInit(void);
void nokia5110_syncBufferPart(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void nokia5110_syncBuffer(void);
void nokia5110_clearBuffer(void);

void nokia5110_bufferWriteString(int16_t x, int16_t y, const char *s);
void nokia5110_bufferSetPixel(int16_t x, int16_t y);
uint8_t nokia5110_bufferGetPixel(int16_t x, int16_t y);
void nokia5110_bufferClearPixel(int16_t x, int16_t y);

#endif 
