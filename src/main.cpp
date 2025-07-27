#include <Arduino.h>
#include <Wire.h>

#include "HP203b.h"

#define LEDR 22
#define LEDG 24
#define LEDB 23

void ledWrite(float r, float g, float b) {
  analogWrite(LEDR, (1.0f - r) * 255);
  analogWrite(LEDG, (1.0f - g) * 255);
  analogWrite(LEDB, (1.0f - b) * 255);
}

HP203B hp;

void setup() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  ledWrite(0.04, 0.04, 0.04);

  Wire.begin();
  Serial.begin(115200);

  // HP203B
  hp.getAddr_HP203B(HP203B_ADDRESS_UPDATED);
  hp.setOSR(OSR_512);
  if (!hp.begin()) {
    while (1) {
      ledWrite(0.1, 0.0, 0.01);
      Serial.println("HP203B not found!");
      delay(1000);
    }
  }
}

void loop() {
  hp.Measure_Altitude();
  Serial.print("alt:");
  Serial.print(hp.hp_sensorData.A);
  Serial.println();
  /*hp.Measure_Pressure();
  Serial.print(",pres:");
  Serial.print(hp.hp_sensorData.P);
  hp.Measure_Temperature();
  Serial.print(",temp:");
  Serial.print(hp.hp_sensorData.T);
  Serial.println();*/

  ledWrite(0.0, 0.1, 0.0);  // Set LED to green
}