#include "display/nokiaLcd/nokia5110.h"
#include "display/nokiaLcd/nokia5110_font.h"
#include "commons/commons.h"

///////////////////////////////////////////////////////

static void nokia5110_gpio_config(void);
static void nokia5110_spi_config(void);

void nokia5110_init(void){
	/* Configure gpio pins */
	nokia5110_gpio_config();
	/* Configure spi pins */
	nokia5110_spi_config();
	/* Set pin initial state */
  nokia5110_light(false);

	NOKIA5110_DC_HIGH(); 		// Mode = command;
	NOKIA5110_DIN_HIGH(); 		// Set In at high level;
	NOKIA5110_CLK_HIGH(); 		// Set CLK high;
	NOKIA5110_CE_HIGH(); 		// Unselect chip;

	/* Reset the LCD to a known state */
	NOKIA5110_RST_LOW();		// Set LCD reset = 0;	
	for (int i = 0; i < 5000; i++); //WTF? we need to use something else.
	NOKIA5110_RST_HIGH();		// LCD_RST = 1

	/* Configure LCD module */
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x21);		// Extended instruction set selected
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0xb7); 		// Set LCD voltage (defined by experimentation...)
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x04);		// Set temperature control (TC2)
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x14);		// Set Bias for 1/48
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x20);		// Revert to standard instruction set
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x0c);		// Set display on in "normal" mode (not inversed)
	nokia5110_clear();											// Clear display (still off)
}
///////////////////////////////////////////////////////

void nokia5110_clear(void) {
	uint8_t y, x;
	nokia5110_gotoXY(0, 0);
	for (y = 0; y < DISPLAY_BANKS; y++) {
		for (x = 0; x < DISPLAY_WIDTH; x++) {
			nokia5110_spi_writeByte(PCD8544_MODE_Data, 0x00);
		}
	}
	nokia5110_gotoXY(0, 0);
}
///////////////////////////////////////////////////////

void nokia5110_gotoXY(int8_t col, int8_t row){
	col *= DISPLAY_BANKS;
  nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x40 | (uint8_t)row);
  nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x80 | (uint8_t)col);
}
///////////////////////////////////////////////////////

void nokia5110_writeChar(char c){
	uint8_t line;
	c -= 32;
	for (line = 0; line < DISPLAY_BANKS; line++) {
		nokia5110_spi_writeByte(PCD8544_MODE_Data, font6_8[(uint8_t)c][line]);
	}
}
///////////////////////////////////////////////////////

void nokia5110_writeString(const char *s){
	while (*s)
		nokia5110_writeChar(*s++);
}
///////////////////////////////////////////////////////

void nokia5110_setContrast(uint8_t contrast){
	/*  LCD Extended Commands. */
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x21);
	/* Set LCD Vop (Contrast). */
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x80 | contrast);
	/*  LCD Standard Commands, horizontal addressing mode. */
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x20);
}
///////////////////////////////////////////////////////

void nokia5110_gpio_config(void){
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable Clocks for GPIO */
	RCC_APB2PeriphClockCmd(PCD8544_GPIO_CLOCK, ENABLE);

	/* Configure LED Pin */
	GPIO_InitStructure.GPIO_Pin = PCD8544_LED_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PCD8544_LED_PORT, &GPIO_InitStructure);

	/* Configure RST Pin */
	GPIO_InitStructure.GPIO_Pin = PCD8544_RST_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PCD8544_RST_PORT, &GPIO_InitStructure);

	/* Configure CE Pin */
	GPIO_InitStructure.GPIO_Pin = PCD8544_CE_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PCD8544_CE_PORT, &GPIO_InitStructure);

	/* Configure DC Pin */
	GPIO_InitStructure.GPIO_Pin = PCD8544_DC_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PCD8544_DC_PORT, &GPIO_InitStructure);
}
///////////////////////////////////////////////////////

void nokia5110_spi_config(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	/* Enable Clocks for GPIO and SPI2 */
	RCC_APB2PeriphClockCmd(PCD8544_GPIO_CLOCK, ENABLE);
	RCC_APB2PeriphClockCmd(PCD8544_SPI_CLOCK, ENABLE);
	// TODO: Modify the structure for SPI1

	/* Configure GPIO Pins for SPI peripheral*/
	GPIO_InitStructure.GPIO_Pin = PCD8544_CLK_PIN | PCD8544_MISO_PIN | PCD8544_MOSI_PIN;
	//GPIO_InitStructure.GPIO_Pin = PCD8544_CLK_PIN | PCD8544_DC_PIN | PCD8544_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(PCD8544_SPI_PORT, &GPIO_InitStructure);

	/* Configure SPI Pins */
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(PCD8544_SPI_PER, &SPI_InitStructure);

	/* Enable SPI Peripheral */
	SPI_Cmd(PCD8544_SPI_PER, ENABLE);
}
///////////////////////////////////////////////////////

void nokia5110_spi_writeByte(PCD8544_MODE_TypeDef mode, uint8_t data){
	NOKIA5110_CE_LOW();			// SPI_CS = 0;
	if(mode == PCD8544_MODE_Command) {
		NOKIA5110_DC_LOW();
	} else {
		NOKIA5110_DC_HIGH();
	}

	/* Loop while DR register in not empty */
	while (SPI_I2S_GetFlagStatus(PCD8544_SPI_PER, SPI_I2S_FLAG_TXE) == RESET);
	/* Send a Byte through the SPI peripheral */
	SPI_I2S_SendData(PCD8544_SPI_PER, data);
	/* Be sure that the character goes to the shift register */
	while (SPI_I2S_GetFlagStatus(PCD8544_SPI_PER, SPI_I2S_FLAG_TXE) == RESET);
	/* Wait until entire byte has been read (which we discard anyway) */
	//	while( SPI_I2S_GetFlagStatus(PCD8544_SPI_PER, SPI_I2S_FLAG_RXNE) != SET );

	NOKIA5110_CE_HIGH();	// SPI_CS = 1;
}
///////////////////////////////////////////////////////

void nokia5110_light(bool on) {
	if (on) NOKIA5110_LED_ON();
	else NOKIA5110_LED_OFF();
}
///////////////////////////////////////////////////////

void nokia5110_setPos(uint8_t page, uint8_t x) {
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x40 | (page & 0x07));
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x80 | x);
}
///////////////////////////////////////////////////////

void nokia5110_printText(int16_t x, int16_t y, const char *value) {
  nokia5110_gotoXY((int8_t)x, (int8_t)y);
	nokia5110_writeString(value);
}

void nokia5110_setPixel(int16_t x, int16_t y) {
  uint8_t bank = (uint8_t)(y >> 3);
  uint8_t bit = (uint8_t)(y - (bank << 3));
  uint8_t c = (uint8_t)(1 << bit);
  nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x40 | (uint8_t)bank);
  nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x80 | (uint8_t)x);
  nokia5110_spi_writeByte(PCD8544_MODE_Data, c);
}

