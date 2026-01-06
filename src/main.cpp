#include <Arduino.h>
#include <Wire.h>

#include "estimator.h"
#include "flash.h"

// Make setup blocking
semaphore_t setup_done;

void setup() {
  sem_init(&setup_done, 0, 1);

  setupHardware();

  // Initialize flash logging
  initFlash();

  // Calibrate gyro bias for 1s
  ledWrite(0.0, 0.0, 0.1);
  unsigned long start = millis();
  hp.startConversion();
  while (millis() - start < 5000) {
    BiasUpdate();
  }

  // Initialize orientation filter
  FilterReset();

  // Indicate setup done
  sem_release(&setup_done);
}

void loop() {
  handleFlashCommands();

  // Get orientation from C matrix
  float roll, pitch, yaw;
  GetOrientation(&roll, &pitch, &yaw);

  // Log all data including motor position/velocity and orientation
  logFlightData(x[0], x[1], x[2], rawSensorData[0], rawSensorData[1], motorpos,
                motorvel, roll, pitch, yaw);

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
  Serial.print(" m, Motor Pos: ");
  Serial.print(motorpos);
  Serial.print(", Motor Vel: ");
  Serial.print(motorvel);
  Serial.print(", Roll: ");
  Serial.print(roll);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Yaw: ");
  Serial.print(yaw);
  Serial.println();

  delay(10);  // 100 Hz = 10ms

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