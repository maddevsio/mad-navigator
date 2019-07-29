#ifndef __SETTINGS_H
#define __SETTINGS_H

#include "stdint.h"
#include "stdbool.h"

#define SETTINGS_MAGIC_V1 0xFBFAF0F9

typedef struct {
    union {
        struct {
            uint32_t magic;
            uint16_t timeShift;
        } values;
        char buffer[124];
    } data;
} SettingsV1;

SettingsV1 settingsReadV1();
bool settingsWriteV1(SettingsV1* settings);
SettingsV1* getCurrentSettings();

#endif