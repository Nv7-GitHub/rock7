/**************************************************************************/
/*
        Distributed with a free-will license.
        Use it any way you want, profit or free, provided it fits in the
   licenses of its associated works. HP203B This code is designed to work with
   the HP203B_I2CS I2C Mini Module available from ControlEverything.com.
        https://shop.controleverything.com/products/precision-barometer-and-altimeter?variant=25687549067#tabs-0-product_tabset-2
*/
/**************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#include "HP203b.h"

/**************************************************************************/
/*
        Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
static uint8_t i2cread(void) {
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

/**************************************************************************/
/*
        Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
static void i2cwrite(uint8_t x) {
#if ARDUINO >= 100
  Wire.write((uint8_t)x);
#else
  Wire.send(x);
#endif
}

/**************************************************************************/
/*
        Writes 8-bits to the destination register
*/
/**************************************************************************/
static void writeRegister(uint8_t i2cAddress, uint8_t reg) {
  Wire.beginTransmission(i2cAddress);
  i2cwrite((uint8_t)reg);
  Wire.endTransmission();
}

/**************************************************************************/
/*
    Reads 24-bits from the specified destination register
*/
/**************************************************************************/
static uint32_t readRegister(uint8_t i2cAddress, uint8_t reg) {
  Wire.beginTransmission(i2cAddress);
  i2cwrite((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, (uint8_t)3);
  return (uint32_t)((int32_t)i2cread() << 16) | ((int32_t)i2cread() << 8) |
         i2cread();
}

/**************************************************************************/
/*
        Instantiates a new HP203B class with appropriate properties
*/
/**************************************************************************/
void HP203B::getAddr_HP203B(uint8_t i2cAddress) {
  hp_i2cAddress = i2cAddress;
  hp_conversionDelay = 17;  // Default for OSR_512, will be updated by setOSR()
}

/**************************************************************************/
/*
        Sets up the Hardware
*/
/**************************************************************************/
bool HP203B::begin() {
  Wire.begin();

  // Reset();
  // delay(hp_conversionDelay);

  return true;
}

/**************************************************************************/
/*
        Sets the Soft Reset Command
        The Sevice will Immediately be Reset No Matter What it is Working On
*/
/**************************************************************************/
void HP203B::Reset() {
  writeRegister(hp_i2cAddress, HP203B_CMD_SOFT_RESET);
  delay(hp_conversionDelay);
}

/**************************************************************************/
/*
        Sets the OSR Command Value to select Decimation Rate of the Internal
   Digital Filter
*/
/**************************************************************************/
void HP203B::setOSR(hpOSR_t osr) {
  hp_osr = osr;

  // Calculate conversion delay based on OSR (Temperature + Pressure/Altitude)
  switch (osr) {
    case OSR_128:
      hp_conversionDelay = 5;  // 4.1ms -> round up to 5ms
      break;
    case OSR_256:
      hp_conversionDelay = 9;  // 8.2ms -> round up to 9ms
      break;
    case OSR_512:
      hp_conversionDelay = 17;  // 16.4ms -> round up to 17ms
      break;
    case OSR_1024:
      hp_conversionDelay = 33;  // 32.8ms -> round up to 33ms
      break;
    case OSR_2048:
      hp_conversionDelay = 66;  // 65.6ms -> round up to 66ms
      break;
    case OSR_4096:
      hp_conversionDelay = 132;  // 131.1ms -> round up to 132ms
      break;
    default:
      hp_conversionDelay = 17;  // Default to OSR_512 timing
      break;
  }
}

/**************************************************************************/
/*
        Gets the OSR Command Value to select Decimation Rate of the Internal
   Digital Filter
 */
/**************************************************************************/
hpOSR_t HP203B::getOSR() { return hp_osr; }

/**************************************************************************/
/*
        Non-blocking interface: Start a conversion
*/
/**************************************************************************/
void HP203B::startConversion(void) {
  // Start conversion for pressure/altitude and temperature
  uint8_t command =
      HP203B_CMD_CONVERT |  // Convert the Sensor Output to the Digital Values
      HP203B_CMD_CHNL_PRESTEMP;  // Sensor Pressure and Temperature Channel

  command |= hp_osr;  // OSR

  writeRegister(hp_i2cAddress, command);
  hp_conversionStartTime = millis();
}

/**************************************************************************/
/*
        Non-blocking interface: Check if conversion is ready
*/
/**************************************************************************/
bool HP203B::isConversionReady(void) {
  return (millis() - hp_conversionStartTime) >= hp_conversionDelay;
}

/**************************************************************************/
/*
        Non-blocking interface: Get conversion delay in ms
*/
/**************************************************************************/
uint8_t HP203B::getConversionDelay(void) { return hp_conversionDelay; }

/**************************************************************************/
/*
        Non-blocking interface: Read all data (P, A, T) after conversion
*/
/**************************************************************************/
void HP203B::readAllData(void) {
  // Read pressure
  uint32_t pressure = readRegister(hp_i2cAddress, HP203B_CMD_READ_P);
  hp_sensorData.P = pressure / 100.0;

  // Read altitude
  uint32_t altitude = readRegister(hp_i2cAddress, HP203B_CMD_READ_A);
  hp_sensorData.A = altitude / 100.0;

  // Read temperature
  uint32_t temperature = readRegister(hp_i2cAddress, HP203B_CMD_READ_T);
  hp_sensorData.T = temperature / 100.0;
}