#include <Arduino.h>
#include <Wire.h>

#include "estimator.h"
#include "flash.h"

// Make setup blocking
semaphore_t setup_done;

void setup() {
  sem_init(&setup_done, 0, 1);

  setupHardware();

  while (!Serial) {
    // wait for Serial to be ready
    delay(10);
  }

  // Initialize flash logging
  initFlash();

  // Calibrate gyro bias for 1s
  ledWrite(0.0, 0.0, 0.1);
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
  handleFlashCommands();
  logFlightData(x[0], x[1], x[2], rawSensorData[0], rawSensorData[1]);

  // Flush buffered data to flash every 10 loops (~200ms at 50Hz)
  static int loopCounter = 0;
  if (++loopCounter >= 10) {
    flushLogBuffer();
    loopCounter = 0;
  }

  // Print to serial for monitoring
  Serial.print("Altitude: ");
  Serial.print(x[0]);
  Serial.print(" m, Velocity: ");
  Serial.print(x[1]);
  Serial.print(" m/s, Accel Bias: ");
  Serial.print(x[2]);
  Serial.print(" m/s^2, Raw Accel: ");
  Serial.print(rawSensorData[0]);
  Serial.print(" m/s^2, Raw Baro: ");
  Serial.print(rawSensorData[1]);
  Serial.println(" m");

  delay(20);  // 50 Hz = 20ms

  // LED indicator: yellow if low storage, green otherwise
  if (checkStorageWarning()) {
    ledWrite(0.1, 0.1, 0.0);  // Yellow warning
  } else {
    ledWrite(0.0, 0.1, 0.0);  // Green normal
  }
}

void setup1() {
  // wait for core0 setup to finish
  sem_acquire_blocking(&setup_done);
}

void loop1() {
  // Update filter
  FilterUpdate();
}