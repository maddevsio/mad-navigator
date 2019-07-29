#ifndef __FLASH_H
#define __FLASH_H

#include "stdbool.h"
#include "stdint.h"

#define BANK1_PAGES  (uint32_t)128
#define BANK1_PAGE_SIZE   (uint32_t)0x400

bool flashInit();
bool flashLock();
bool flashUnlock();
bool flashRead(uint32_t startAddress, void* buffer, uint16_t size);
bool flashWrite(uint32_t startAddress, void* buffer, uint16_t size);

#endif //__FLASH_H
