/*
 * test.ino
 *
 * Simple sketch to test & demo the chip
 *
 * Author: Sean Caulfield <sean@yak.net>
 * License: GPLv2.0
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include <APDS9250.h>

APDS9250 apds9250 = APDS9250();
bool found = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!apds9250.begin()) {
    found = true;
    Serial.println("APDS9250 found!");
  } else {
    Serial.println("APDS9250 not found! Check wiring?");
  }
}

void loop() {
  delay(1000);
}
