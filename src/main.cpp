#include <Arduino.h>
#include <Wire.h>

#include "hardware.h"

void setup() {
  setupHardware();

  // Initialize filtAz
  extern float filtAz;
  float ax, ay, az;
  mpu.readSensor();
  mpu.getAccel(ax, ay, az);
  filtAz = az;

  // Start first HP203B conversion
  hp.startConversion();
}

float filtAz = 0.0f;
unsigned long lastPrintTime = 0;

unsigned long readTime;
void loop() {
  // Non-blocking HP203B read
  unsigned long start = micros();
  if (hp.isConversionReady()) {
    hp.readAllData();
    hp.startConversion();  // Start next conversion
    readTime = micros() - start;
  }
  start = micros();

  // Fast loop: Read IMU and update ODrive continuously
  float ax, ay, az;
  mpu.readSensor();
  mpu.getAccel(ax, ay, az);

  float gx, gy, gz;
  mpu.getGyro(gx, gy, gz);

  unsigned long delTime = micros() - start;

  // Update filtered Az and ODrive position
  filtAz = 0.9f * filtAz + 0.1f * az;
  odrv.setPosition(filtAz * 10.0f);

  // Print data every 50ms
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= 50) {
    lastPrintTime = currentTime;

    Serial.print("alt:");
    Serial.print(hp.hp_sensorData.A);
    Serial.print(",pres:");
    Serial.print(hp.hp_sensorData.P);
    Serial.print(",temp:");
    Serial.print(hp.hp_sensorData.T);
    Serial.print(",ax:");
    Serial.print(ax);
    Serial.print(",ay:");
    Serial.print(ay);
    Serial.print(",az:");
    Serial.print(az);
    Serial.print(",gx:");
    Serial.print(gx);
    Serial.print(",gy:");
    Serial.print(gy);
    Serial.print(",gz:");
    Serial.print(gz);
    Serial.print(",mpos:");
    Serial.print(motorpos);
    Serial.print(",mvel:");
    Serial.print(motorvel);
    Serial.print(",filtaz:");
    Serial.print(filtAz);
    Serial.print(",dt:");
    Serial.print(delTime + readTime);
    Serial.print(",rdt:");
    Serial.print(readTime);
    Serial.print(",ldt:");
    Serial.print(delTime);
    Serial.println();

    ledWrite(0.0, 0.1, 0.0);  // Set LED to green
  }
}