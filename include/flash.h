#ifndef FLASH_H
#define FLASH_H

#include <Arduino.h>

void initFlash();
void logFlightData(float altitude, float velocity, float accelBias,
                   float rawAccel, float rawBaro);
void flushLogBuffer();  // Flush RAM buffer to flash
void handleFlashCommands();
bool checkStorageWarning();  // Returns true if low storage warning is active

#endif  // FLASH_H
