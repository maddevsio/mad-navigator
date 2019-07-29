/*******************************************************************************
* File  		:	stm32f10x_pcd8544.h
* Description	: 	STM32F10x library for NOKIA 5110 LCD driver, PCD8544
* Author		: 	Aytac Dilek
* Note			: 	GNU General Public License, version 3 (GPL-3.0)
*******************************************************************************/
#ifndef __STM32F10X_PCD8544_H
#define __STM32F10X_PCD8544_H

/* Includes ********************************************************************/
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include <stdbool.h>

/* Defines *********************************************************************/
#define PCD8544_SPI_PER			SPI1
#define PCD8544_SPI_PORT		GPIOA
#define PCD8544_SPI_CLOCK		RCC_APB2Periph_SPI1
#define PCD8544_GPIO_PORT		GPIOA
#define PCD8544_GPIO_CLOCK		RCC_APB2Periph_GPIOA

#define PCD8544_LED_PORT 		GPIOA
#define PCD8544_LED_PIN 		GPIO_Pin_2
#define PCD8544_RST_PORT 		GPIOA
#define PCD8544_RST_PIN 		GPIO_Pin_1
#define PCD8544_CE_PORT			GPIOA
#define PCD8544_CE_PIN 			GPIO_Pin_4
#define PCD8544_DC_PORT 		GPIOA
#define PCD8544_DC_PIN 			GPIO_Pin_3
#define PCD8544_CLK_PORT		GPIOA
#define PCD8544_CLK_PIN			GPIO_Pin_5
#define PCD8544_MISO_PORT		GPIOA
#define PCD8544_MISO_PIN		GPIO_Pin_6
#define PCD8544_MOSI_PORT		GPIOA
#define PCD8544_MOSI_PIN		GPIO_Pin_7

#define PCD8544_WIDTH			84
#define PCD8544_HEIGHT			48

/* Macros **********************************************************************/
#define NOKIA5110_LED_ON()		GPIO_SetBits(PCD8544_LED_PORT, PCD8544_LED_PIN)
#define NOKIA5110_LED_OFF()		GPIO_ResetBits(PCD8544_LED_PORT, PCD8544_LED_PIN)
#define NOKIA5110_RST_LOW()		GPIO_ResetBits(PCD8544_RST_PORT, PCD8544_RST_PIN)
#define NOKIA5110_RST_HIGH()	GPIO_SetBits(PCD8544_RST_PORT, PCD8544_RST_PIN)
#define NOKIA5110_DC_LOW()		GPIO_ResetBits(PCD8544_DC_PORT, PCD8544_DC_PIN)
#define NOKIA5110_DC_HIGH()		GPIO_SetBits(PCD8544_DC_PORT, PCD8544_DC_PIN)
#define NOKIA5110_DIN_LOW()		GPIO_ResetBits(PCD8544_MOSI_PORT, PCD8544_MOSI_PIN)
#define NOKIA5110_DIN_HIGH()	GPIO_SetBits(PCD8544_MOSI_PORT, PCD8544_MOSI_PIN)
#define NOKIA5110_CLK_LOW()		GPIO_ResetBits(PCD8544_CLK_PORT, PCD8544_CLK_PIN)
#define NOKIA5110_CLK_HIGH()	GPIO_SetBits(PCD8544_CLK_PORT, PCD8544_CLK_PIN)
#define NOKIA5110_CE_LOW()		GPIO_ResetBits(PCD8544_CE_PORT, PCD8544_CE_PIN)
#define NOKIA5110_CE_HIGH()		GPIO_SetBits(PCD8544_CE_PORT, PCD8544_CE_PIN)

/* Enumerations ****************************************************************/
typedef enum{
	PCD8544_MODE_Command = 0,
	PCD8544_MODE_Data = 1,
}PCD8544_MODE_TypeDef;

#define DISPLAY_BANKS 6
#define DISPLAY_WIDTH 84

///////////////////////////////////////////////////////

/* Global Functions ************************************************************/

void nokia5110_init(void);
void nokia5110_clear(void);
void nokia5110_gotoXY(int8_t col, int8_t row);
void nokia5110_writeChar(char c);
void nokia5110_writeString(const char *s);
void nokia5110_setContrast(uint8_t contrast);
void nokia5110_light(bool on);
void nokia5110_printText(int16_t x, int16_t y, const char *value);
void nokia5110_setPixel(int16_t x, int16_t y);

void nokia5110_spi_writeByte(PCD8544_MODE_TypeDef mode, uint8_t data);

#endif // __STM32F10X_PCD8544_H
