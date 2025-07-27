#include <Arduino.h>

#define LEDR 22
#define LEDG 24
#define LEDB 23

void ledWrite(float r, float g, float b) {
  analogWrite(LEDR, (1.0f - r) * 255);
  analogWrite(LEDG, (1.0f - g) * 255);
  analogWrite(LEDB, (1.0f - b) * 255);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  ledWrite(0.04, 0.04, 0.04);  // Set LED to red
}

void loop() {
  Serial.println("Hello, Rock V7!");
  delay(1000);
  ledWrite(0.0, 0.1, 0.0);  // Set LED to green
}