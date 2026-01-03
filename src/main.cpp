#include <Arduino.h>
#include <Wire.h>

#include "estimator.h"

// Make setup blocking
semaphore_t setup_done;

void setup() {
  sem_init(&setup_done, 0, 1);

  setupHardware();
  unsigned long start = millis();

  while (!Serial) {
    // wait for Serial to be ready
    delay(10);
  }

  // Initialize orientation filter
  FilterReset();

  // Indicate setup done
  sem_release(&setup_done);
}

void loop() {
  // Print data every 50ms
  Serial.print("ax:");
  Serial.print(aGlob[0], 3);
  Serial.print(",ay:");
  Serial.print(aGlob[1], 3);
  Serial.print(",az:");
  Serial.print(aGlob[2], 3);
  Serial.println();
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