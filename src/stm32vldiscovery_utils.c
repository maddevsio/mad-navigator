/*
 * stm32vldiscovery_utils.c
 *
 *  Created on: Apr 22, 2019
 *      Author: lezh1k
 */
#include "stm32vldiscovery_utils.h"

volatile uint32_t *DWT_CONTROL = (uint32_t *) 0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *) 0xE0001004;
volatile uint32_t *DEMCR = (uint32_t *) 0xE000EDFC;
volatile uint32_t *LAR  = (uint32_t *) 0xE0001FB0;   // <-- added lock access register

void initDWT(void) {
		 *DEMCR = *DEMCR | 0x01000000;     // enable trace
		 *LAR = 0xC5ACCE55;                // <-- added unlock access to DWT (ITM, etc.)registers
		 *DWT_CYCCNT = 0;                  // clear DWT cycle counter
		 *DWT_CONTROL = *DWT_CONTROL | 1;  // enable DWT cycle counter
}
