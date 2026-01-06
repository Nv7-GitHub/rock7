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
  // initFlash();

  // Calibrate gyro bias for 1s
  /*ledWrite(0.0, 0.0, 0.1);
  unsigned long start = millis();
  hp.startConversion();
  while (millis() - start < 5000) {
    BiasUpdate();
  }

  // Initialize orientation filter
  FilterReset();*/

  // Indicate setup done
  sem_release(&setup_done);

  // ODrive
  EnableOdrv();
  odrv.setPosition(0);
}

void loop() {
  Serial.printf("motorpos: %f, motorvel: %f\n", motorpos, motorvel);
  delay(100);
  ledWrite(0.0, 0.1, 0.0);

  // Ask for input and echo typed characters until Enter is pressed
  Serial.print("Enter position setpoint (float): ");

  // Clear any stray input before we start
  while (Serial.available() > 0) {
    Serial.read();
    delay(1);
  }

  String input = "";
  bool done = false;
  while (!done) {
    if (Serial.available() > 0) {
      char c = (char)Serial.read();
      // Handle CR or LF as Enter
      if (c == '\r' || c == '\n') {
        // If CR was received, eat a following LF if present
        if (c == '\r' && Serial.peek() == '\n') {
          Serial.read();
        }
        Serial.println();  // move to next line in monitor
        done = true;
        break;
      }

      // Handle backspace or delete
      if (c == 8 || c == 127) {
        if (input.length() > 0) {
          input.remove(input.length() - 1);
          // erase last char on terminal: backspace, space, backspace
          Serial.write(8);
          Serial.write(' ');
          Serial.write(8);
        }
        continue;
      }

      // Normal printable character: append and echo
      input += c;
      Serial.write(c);
    } else {
      delay(10);
    }
  }

  float pos_setpoint = input.toFloat();
  Serial.printf("Setting position setpoint to %f\n", pos_setpoint);
  odrv.setPosition(pos_setpoint);
}

void setup1() {
  // wait for core0 setup to finish
  sem_acquire_blocking(&setup_done);
}

void loop1() {
  // Update filter
  // FilterUpdate();
}