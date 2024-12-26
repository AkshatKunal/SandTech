#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void InitVL53L0X() {
  if (!lox.begin()) while (1);  // Halt execution if initialization fails
}

float Dist(float correction = 0.0) {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  return measure.RangeStatus != 4 ? (measure.RangeMilliMeter / 10.0) + correction : -1.0;
}

void forward() {
//hello akshad
}

void rotate(int angle) {
 // here also akshad
}

void setup() {
  Serial.begin(9600);
  InitVL53L0X();
}

void loop() {
  while (Dist() > 50) { 
    forward();
  }
  rotate(90);
}