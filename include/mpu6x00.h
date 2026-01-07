/*
   Class definition for MPU6000, MPU6500 IMUs using SPI bus

   Copyright (c) 2023 Simon D. Levy

   MIT License
 */

#pragma once

#include <SPI.h>
#include <stdint.h>

#include "config.h"

class Mpu6x00 {
 public:
  typedef enum {

    GYRO_250DPS,
    GYRO_500DPS,
    GYRO_1000DPS,
    GYRO_2000DPS

  } gyroFsr_e;

  typedef enum {

    ACCEL_2G,
    ACCEL_4G,
    ACCEL_8G,
    ACCEL_16G

  } accelFsr_e;

  typedef enum {

    DLPF_260HZ,  // 260 Hz bandwidth, no filtering (DLPF disabled)
    DLPF_184HZ,  // 184 Hz bandwidth
    DLPF_92HZ,   // 92 Hz bandwidth
    DLPF_41HZ,   // 41 Hz bandwidth (good balance)
    DLPF_20HZ,   // 20 Hz bandwidth (smooth)
    DLPF_10HZ,   // 10 Hz bandwidth (very smooth)
    DLPF_5HZ     // 5 Hz bandwidth (extremely smooth)

  } dlpf_e;

  /**
   * Set the Output Data Rate (ODR) in Hz.
   * Valid rates: 1000, 500, 250, 200, 125, 100, 50, 25, 10, 5, 4 Hz
   * Note: ODR only works when DLPF is enabled (DLPF_CFG >= 1)
   * If DLPF is disabled, sensor runs at 8 kHz
   */
  void setODR(uint16_t odr_hz) {
    // ODR = 1000 / (1 + SMPLRT_DIV)
    // SMPLRT_DIV = (1000 / ODR) - 1
    if (odr_hz == 0 || odr_hz > 1000) {
      odr_hz = 1000;  // Clamp to valid range
    }
    uint8_t div = (1000 / odr_hz) - 1;
    writeRegister(REG_SMPLRT_DIV, div);
    delayMicroseconds(15);
  }

  /**
   * Set the Digital Low Pass Filter (DLPF) configuration.
   */
  void setDLPF(dlpf_e dlpf) {
    m_dlpf = dlpf;
    writeRegister(REG_CONFIG, (uint8_t)dlpf);
    delayMicroseconds(15);
  }

  /**
   * Returns true on success, false on failure.
   */
  bool begin(void) {
    pinMode(m_csPin, OUTPUT);

    writeRegister(REG_PWR_MGMT_1, BIT_RESET);
    delay(100);

    writeRegister(REG_PWR_MGMT_1, BIT_CLK_SEL_PLLGYROZ);
    delayMicroseconds(7);

    writeRegister(REG_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(15);

    writeRegister(REG_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    writeRegister(REG_SMPLRT_DIV, 0);
    delayMicroseconds(15);

    if (whoAmI() != m_deviceId) {
      Serial.println("ID mismatch, expected " + String(m_deviceId) + ", got " +
                     String(whoAmI()));
      return false;
    }

    writeRegister(REG_GYRO_CONFIG, (uint8_t)(m_gyroFsr << 3));
    delayMicroseconds(15);

    writeRegister(REG_ACCEL_CONFIG, (uint8_t)(m_accelFsr << 3));
    delayMicroseconds(15);

    writeRegister(REG_INT_PIN_CFG, 0x10);
    delayMicroseconds(15);

    writeRegister(REG_INT_ENABLE, BIT_RAW_RDY_EN);
    delayMicroseconds(15);

    writeRegister(REG_CONFIG, (uint8_t)m_dlpf);
    delayMicroseconds(1);

    return true;
  }

  void readSensor(void) {
    readRegisters(REG_ACCEL_XOUT_H, m_buffer, 14, SPI_FULL_CLK_HZ);
  }

  void getGyro(float& gx, float& gy, float& gz) {
    gx = getRawValue(9) * m_gyroScale;
    gy = getRawValue(11) * m_gyroScale;
    gz = getRawValue(13) * m_gyroScale;
  }

  void getAccel(float& ax, float& ay, float& az) {
    ax = getRawValue(1) * m_accelScale;
    ay = getRawValue(3) * m_accelScale;
    az = getRawValue(5) * m_accelScale;
  }

  float getAccelX(void) { return getAccelValue(1); }

  float getAccelY(void) { return getAccelValue(3); }

  float getAccelZ(void) { return getAccelValue(5); }

  float getGyroX(void) { return getGyroValue(9); }

  float getGyroY(void) { return getGyroValue(11); }

  float getGyroZ(void) { return getGyroValue(13); }

  int16_t getRawAccelX(void) { return getRawValue(1); }

  int16_t getRawAccelY(void) { return getRawValue(3); }

  int16_t getRawAccelZ(void) { return getRawValue(5); }

  int16_t getRawGyroX(void) { return getRawValue(9); }

  int16_t getRawGyroY(void) { return getRawValue(11); }

  int16_t getRawGyroZ(void) { return getRawValue(13); }

 protected:
  Mpu6x00(const uint8_t deviceId, SPIClass& spi, const uint8_t csPin,
          const gyroFsr_e gyroFsr, const accelFsr_e accelFsr) {
    init(deviceId, &spi, csPin, gyroFsr, accelFsr);
  }

  Mpu6x00(const uint8_t deviceId, const uint8_t csPin, const gyroFsr_e gyroFsr,
          const accelFsr_e accelFsr = ACCEL_16G) {
    init(deviceId, &SPI, csPin, gyroFsr, accelFsr);
  }

 private:
  // Configuration bits
  static const uint8_t BIT_RAW_RDY_EN = 0x01;
  static const uint8_t BIT_CLK_SEL_PLLGYROZ = 0x03;
  static const uint8_t BIT_I2C_IF_DIS = 0x10;
  static const uint8_t BIT_RESET = 0x80;

  // Registers
  static const uint8_t REG_SMPLRT_DIV = 0x19;
  static const uint8_t REG_CONFIG = 0x1A;
  static const uint8_t REG_GYRO_CONFIG = 0x1B;
  static const uint8_t REG_ACCEL_CONFIG = 0x1C;
  static const uint8_t REG_INT_PIN_CFG = 0x37;
  static const uint8_t REG_INT_ENABLE = 0x38;
  static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
  static const uint8_t REG_USER_CTRL = 0x6A;
  static const uint8_t REG_PWR_MGMT_1 = 0x6B;
  static const uint8_t REG_PWR_MGMT_2 = 0x6C;
  static const uint8_t REG_WHO_AM_I = 0x75;

  static const uint32_t SPI_FULL_CLK_HZ = 20000000;
  static const uint32_t SPI_INIT_CLK_HZ = 1000000;

  SPIClass* m_spi;

  uint8_t m_csPin;

  uint8_t m_deviceId;

  gyroFsr_e m_gyroFsr;
  accelFsr_e m_accelFsr;
  dlpf_e m_dlpf;

  float m_gyroScale;
  float m_accelScale;

  uint8_t m_buffer[15];

  void init(const uint8_t deviceId, SPIClass* spi, const uint8_t csPin,
            const gyroFsr_e gyroFsr, const accelFsr_e accelFsr) {
    m_deviceId = deviceId;
    m_spi = spi;
    m_csPin = csPin;

    m_gyroFsr = gyroFsr;
    m_accelFsr = accelFsr;
    m_dlpf = DLPF_260HZ;  // Default: DLPF disabled (fastest, 8kHz)

    // gyro full-scale values in degrees/sec for each enum entry
    float gscale[] = {250.0f, 500.0f, 1000.0f, 2000.0f};
    // convert to rad/s: (deg/s) / 32768 * (PI/180)
    m_gyroScale =
        (gscale[gyroFsr] / 32768.0f) * (3.14159265358979323846f / 180.0f);

    // accel full-scale values in g for each enum entry
    float ascale[] = {2.0f, 4.0f, 8.0f, 16.0f};
    // convert to m/s^2: (g) / 32768 * G
    m_accelScale = (ascale[accelFsr] / 32768.0f) * G;
  }

  void writeRegister(const uint8_t reg, const uint8_t val) {
    m_spi->beginTransaction(SPISettings(SPI_INIT_CLK_HZ, MSBFIRST, SPI_MODE3));

    digitalWrite(m_csPin, LOW);
    m_spi->transfer(reg);
    m_spi->transfer(val);
    digitalWrite(m_csPin, HIGH);

    m_spi->endTransaction();
  }

  void readRegisters(const uint8_t addr, uint8_t* buffer, const uint8_t count,
                     const uint32_t spiClkHz) {
    m_spi->beginTransaction(SPISettings(spiClkHz, MSBFIRST, SPI_MODE3));

    digitalWrite(m_csPin, LOW);

    buffer[0] = addr | 0x80;
    m_spi->transfer(buffer, count + 1);

    digitalWrite(m_csPin, HIGH);

    m_spi->endTransaction();
  }

  uint8_t whoAmI(void) {
    uint8_t buf[2] = {};
    readRegisters(REG_WHO_AM_I, buf, 1, SPI_INIT_CLK_HZ);
    return buf[1];
  }

  int16_t getRawValue(const uint8_t offset) {
    return (((int16_t)m_buffer[offset]) << 8) | m_buffer[offset + 1];
  }

  float getAccelValue(const uint8_t k) {
    return getFloatValue(k, m_accelScale);
  }

  float getGyroValue(const uint8_t k) { return getFloatValue(k, m_gyroScale); }

  float getFloatValue(const uint8_t k, const float scale) {
    return getRawValue(k) * scale;
  }

};  // class Mpu6x00

class Mpu6000 : public Mpu6x00 {
 public:
  Mpu6000(SPIClass& spi, const uint8_t csPin,
          const gyroFsr_e gyroFsr = GYRO_2000DPS,
          const accelFsr_e accelFsr = ACCEL_16G)
      : Mpu6x00(0x68, spi, csPin, gyroFsr, accelFsr) {}

  Mpu6000(const uint8_t csPin, const gyroFsr_e gyroFsr = GYRO_2000DPS,
          const accelFsr_e accelFsr = ACCEL_16G)
      : Mpu6x00(0x68, SPI, csPin, gyroFsr, accelFsr) {}
};

class Mpu6500 : public Mpu6x00 {
 public:
  Mpu6500(SPIClass& spi, const uint8_t csPin,
          const gyroFsr_e gyroFsr = GYRO_2000DPS,
          const accelFsr_e accelFsr = ACCEL_16G)
      : Mpu6x00(0x70, spi, csPin, gyroFsr, accelFsr) {}

  Mpu6500(const uint8_t csPin, const gyroFsr_e gyroFsr = GYRO_2000DPS,
          const accelFsr_e accelFsr = ACCEL_16G)
      : Mpu6x00(0x70, SPI, csPin, gyroFsr, accelFsr) {}
};
