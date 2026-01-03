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
  hp.startConversion();
  while (millis() - start < 1000) {
    BiasUpdate();
  }

  // Initialize orientation filter
  FilterReset();

  // Indicate setup done
  sem_release(&setup_done);
}

void loop() {
  Serial.print("Altitude: ");
  Serial.print(x[0]);
  Serial.print(" m, Velocity: ");
  Serial.print(x[1]);
  Serial.print(" m/s, Accel Bias: ");
  Serial.print(x[2]);
  Serial.println(" m/s^2");
  delay(50);
}

void setup1() {
  // wait for core0 setup to finish
  sem_acquire_blocking(&setup_done);
}

void loop1() {
  // Update filter
  FilterUpdate();
}