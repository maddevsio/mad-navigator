#include "flash/settings.h"
#include "flash/flash.h"

#define SETTINGS_OFFSET (BANK1_PAGES - 1) * BANK1_PAGE_SIZE
static SettingsV1 currentSettings;


SettingsV1* getCurrentSettings() {
    if(currentSettings.data.values.magic != SETTINGS_MAGIC_V1) {
        currentSettings = settingsReadV1();
    }
    return &currentSettings;
}

SettingsV1 defaultSettingsV1() {
    SettingsV1 result;
    memset(&result, 0, sizeof(SettingsV1));
    //result.data.values.magic != SETTINGS_MAGIC_V1;
    return result;
}

SettingsV1 settingsReadV1() {
    SettingsV1 result;
    memset(&result, 0, sizeof(SettingsV1));
    if(!flashInit()) 
        return result;

    if(!flashRead(SETTINGS_OFFSET, &result, sizeof(SettingsV1))) {
        memset(&result, 0, sizeof(SettingsV1));
    }

    if(result.data.values.magic != SETTINGS_MAGIC_V1) {
        result = defaultSettingsV1();
    }

    currentSettings = result;
    return result;
}

bool settingsWriteV1(SettingsV1* settings) {
    if(!flashInit())
        return false;
    flashUnlock();
    settings->data.values.magic = SETTINGS_MAGIC_V1;
    bool result = flashWrite(SETTINGS_OFFSET, settings, sizeof(SettingsV1));
    flashLock();
    return result;
}