#include "flash/flash.h"
#include "stm32f10x_flash.h"

#define BANK1_START_ADDR  ((uint32_t)0x08008000)
#define BANK1_END_ADDR    (BANK1_START_ADDR + (uint32_t)(BANK1_PAGES * BANK1_PAGE_SIZE))

volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;

bool flashInit() {
    return true;
}

bool flashLock() {
    FLASH_LockBank1();
}

bool flashUnlock() {
    FLASH_UnlockBank1();
}

bool flashRead(uint32_t startAddress, void* buffer, uint16_t size) {
    if(BANK1_START_ADDR + startAddress + size > BANK1_END_ADDR) {
        size = BANK1_END_ADDR - (BANK1_START_ADDR + startAddress);
    }
    memcpy(buffer, BANK1_START_ADDR + startAddress, size);
}

bool flashWrite(uint32_t startAddress, void* buffer, uint16_t size) {
    uint32_t endAddress = BANK1_START_ADDR + startAddress + size;
    uint16_t startPage = (BANK1_START_ADDR + startAddress) / BANK1_PAGE_SIZE;
    uint16_t endPage = (BANK1_START_ADDR + startAddress + size) / BANK1_PAGE_SIZE;
    flashUnlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
    for(uint16_t page = 0; page <= (endPage - startPage); page++) {
        FLASHStatus = FLASH_ErasePage(BANK1_START_ADDR + (BANK1_PAGE_SIZE * page));
    }

    uint32_t *data = buffer;
    for(uint32_t address = BANK1_START_ADDR + startAddress; (address < endAddress) && (FLASHStatus == FLASH_COMPLETE); address += 4) {
        FLASHStatus = FLASH_ProgramWord(address, data);
        data++;
    }

    flashLock();
}
