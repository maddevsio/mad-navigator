/*
 * stm32f10x_pcd8544_buffered.c
 *
 *  Created on: Apr 29, 2019
 *      Author: raistlin
 */

#include <string.h>
#include "display/nokiaLcd/nokia5110_buffered.h"
#include "display/nokiaLcd/nokia5110.h"
#include "commons/commons.h"

static uint8_t screenBuffer[DISPLAY_WIDTH][DISPLAY_BANKS];
static void nokia5110_bufferWriteChar(int8_t x, int8_t y, char c);

void nokia5110_bufferInit(void){
	memset(screenBuffer, 0, DISPLAY_WIDTH * DISPLAY_BANKS);
	nokia5110_init();
}
///////////////////////////////////////////////////////

void nokia5110_syncBufferPart(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  for(int16_t y = y0; y < y1; y++) {
//    nokia5110_gotoXY((int8_t)y, (int8_t)x0);
    nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x40 | (uint8_t)y);
    nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x80 | (uint8_t)x0);

    for(int16_t x = x0; x < x1; x++) {
			nokia5110_spi_writeByte(PCD8544_MODE_Data, screenBuffer[x][y]);
		}
	}
}
///////////////////////////////////////////////////////

void nokia5110_syncBuffer() {
	nokia5110_syncBufferPart(0, 0, DISPLAY_WIDTH, DISPLAY_BANKS);
}
///////////////////////////////////////////////////////

void nokia5110_clearBuffer() {
	memset(screenBuffer, 0, DISPLAY_WIDTH * DISPLAY_BANKS);
}
///////////////////////////////////////////////////////

void nokia5110_bufferWriteString(int16_t x, int16_t y, const char *s) {
	uint8_t i = 0;
  for (; *s; ++s, ++i) {
    nokia5110_bufferWriteChar((int8_t)((x+i)*DISPLAY_BANKS), (int8_t)y, *s);
  }
}
///////////////////////////////////////////////////////

void nokia5110_bufferWriteChar(int8_t x, int8_t y, char c) {
  char tmp = c - 0x20;
  for(int8_t i = 0; i < DISPLAY_BANKS; i++)
    screenBuffer[x + i][y] = font6_8[(uint8_t)tmp][i];
}
///////////////////////////////////////////////////////

void nokia5110_bufferSetPixel(int16_t x, int16_t y) {
  uint8_t bank = (uint8_t)(y >> 3);
  uint8_t bit = (uint8_t)(y - (bank << 3));
  uint8_t c = (uint8_t)((1 << bit) | screenBuffer[x][bank]);
	screenBuffer[x][bank] = c;
}
///////////////////////////////////////////////////////

void nokia5110_bufferClearPixel(int16_t x, int16_t y) {
  uint8_t bank = (uint8_t)(y >> 3);
  uint8_t bit = (uint8_t)(y - (bank << 3));
  uint8_t c = (0xff ^ (1 << bit)) & screenBuffer[x][bank];
	screenBuffer[x][bank] = c;
}
///////////////////////////////////////////////////////

uint8_t nokia5110_bufferGetPixel(int16_t x, int16_t y) {
  uint8_t bank = (uint8_t)y >> 3;
  uint8_t bit = (uint8_t)(y - (bank << 3));
  return screenBuffer[x][bank] & (1 << bit);
//  uint8_t c = screenBuffer[x][bank] >> bit;
//	return c & 1;
}
///////////////////////////////////////////////////////
