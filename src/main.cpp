#include <Arduino.h>
#include <Wire.h>

#include "HP203b.h"
#include "mpu6x00.h"

#define LEDR 22
#define LEDG 24
#define LEDB 23
#define IMU_SCK 10
#define IMU_MOSI 11
#define IMU_MISO 12
#define IMU_CS 13

void ledWrite(float r, float g, float b) {
  analogWrite(LEDR, (1.0f - r) * 255);
  analogWrite(LEDG, (1.0f - g) * 255);
  analogWrite(LEDB, (1.0f - b) * 255);
}

HP203B hp;
Mpu6500 mpu(SPI1, IMU_CS);

void setup() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  ledWrite(0.04, 0.04, 0.04);

  Wire.begin();

  SPI.begin(false);

  SPI1.setSCK(IMU_SCK);
  SPI1.setMOSI(IMU_MOSI);
  SPI1.setMISO(IMU_MISO);
  SPI1.begin(false);

  Serial.begin(115200);

  // HP203B
  hp.getAddr_HP203B(HP203B_ADDRESS_UPDATED);
  hp.setOSR(OSR_512);
  if (!hp.begin()) {
    while (1) {
      ledWrite(0.1, 0.0, 0.0);
      Serial.println("HP203B not found!");
      delay(1000);
    }
  }

  // MPU6500
  while (!mpu.begin()) {
    ledWrite(0.1, 0.0, 0.0);
    Serial.println("MPU6500 not found!");
    delay(1000);
  }
}

void loop() {
  /*hp.Measure_Altitude();
  Serial.print("alt:");
  Serial.print(hp.hp_sensorData.A);
  hp.Measure_Pressure();
  Serial.print(",pres:");
  Serial.print(hp.hp_sensorData.P);
  hp.Measure_Temperature();
  Serial.print(",temp:");
  Serial.print(hp.hp_sensorData.T);
  Serial.println();*/

  float ax, ay, az;
  mpu.readSensor();
  mpu.getAccel(ax, ay, az);
  Serial.print("ax:");
  Serial.print(ax);
  Serial.print(",ay:");
  Serial.print(ay);
  Serial.print(",az:");
  Serial.print(az);
  float gx, gy, gz;
  mpu.getGyro(gx, gy, gz);
  Serial.print(",gx:");
  Serial.print(gx);
  Serial.print(",gy:");
  Serial.print(gy);
  Serial.print(",gz:");
  Serial.println(gz);
  delay(50);

  ledWrite(0.0, 0.1, 0.0);  // Set LED to green
}