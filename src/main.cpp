#include <Arduino.h>
#include <Wire.h>

#include "estimator.h"

// Make setup blocking
semaphore_t setup_done;

void setup() {
  sem_init(&setup_done, 0, 1);

  setupHardware();

  while (!Serial) {
    // wait for Serial to be ready
    delay(10);
  }

  // Calibrate gyro bias for 1s
  ledWrite(0.1, 0.0, 0.0);
  unsigned long start = millis();
  while (millis() - start < 1000) {
    OrientationBiasUpdate();
  }

  // Initialize orientation filter
  FilterReset();

  // Indicate setup done
  sem_release(&setup_done);
}

void loop() {
  // Calculate stdev
  float avgAz = 0.0f;
  float rmsAz = 0.0f;
  for (int i = 0; i < 1000; i++) {
    delayMicroseconds(2000);
    rmsAz += aGlob[2] * aGlob[2];
    avgAz += aGlob[2];
  }
  avgAz /= 1000.0f;
  rmsAz = sqrtf(rmsAz / 1000.0f - avgAz * avgAz);
  Serial.print("Accel Z stdev:");
  Serial.print(rmsAz, 6);
  Serial.print(", mean:");
  Serial.println(avgAz, 6);
}

void setup1() {
  // wait for core0 setup to finish
  sem_acquire_blocking(&setup_done);
}

void loop1() {
  // Update filter
  FilterUpdate();
}