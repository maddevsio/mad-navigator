#ifndef __STM32F10xVLDISCOVERY_RCC_H
#define __STM32F10xVLDISCOVERY_RCC_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t *DWT_CONTROL;
extern volatile uint32_t *DWT_CYCCNT;
extern volatile uint32_t *DEMCR;
extern volatile uint32_t *LAR;

void initDWT(void);


#ifdef __cplusplus
}

#endif //__cplusplus
#endif //__STM32F10xVLDISCOVERY_RCC_H
