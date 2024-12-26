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


#define Motor1a 8
#define Motor1b 9
#define Motor2a 7
#define Motor2b 4
const int ENA = 5; 
const int ENB = 6;

#define encoderOne 2
#define encoderTwo 3

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;

const int ticksPerRevolution = 260;
const float wheelBaseCircumference = 2 * 3.14159 * 9;
const float wheelCircumference = 2 * 3.14159 * 3.25;

void setup() {
  Serial.begin(9600);
  pinMode(Motor1a, OUTPUT);
  pinMode(Motor1b, OUTPUT);
  pinMode(Motor2a, OUTPUT);
  pinMode(Motor2b, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderOne), incEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderTwo), incEncoder2, RISING);
  // stopMotors();

  InitVL53L0X();
}

void loop() {
  rotate(90);
  delay(1000);
  rotate(90);
  delay(1000);
  forward(100);
  delay(1000);
}

void incEncoder1() {
  encoderCount1++;
}

void incEncoder2() {
  encoderCount2++;
}

void rotate(int angle) {
  float rotationDistance = (angle / 360.0) * wheelBaseCircumference;
  int targetTicks = (rotationDistance / wheelCircumference) * ticksPerRevolution * 0.8;
  encoderCount1 = 0;
  encoderCount2 = 0;
  digitalWrite(Motor1a, HIGH);
  digitalWrite(Motor1b, LOW);
  digitalWrite(Motor2a, LOW);
  digitalWrite(Motor2b, HIGH);
  analogWrite(ENA, 120);
  analogWrite(ENB, 120);
  while (abs(encoderCount1) < targetTicks && abs(encoderCount2) < targetTicks) {
    Serial.print("Encoder1: ");
    Serial.print(encoderCount1);
    Serial.print(" Encoder2: ");
    Serial.println(encoderCount2);
    if (abs(encoderCount1 - targetTicks) < 30){
        analogWrite(ENA, 80);
        analogWrite(ENB, 80);
    }
  }
  stopMotors();
  encoderCount1 = 0;
  encoderCount2 = 0;
}

void forward(int distance) {
  int speed = 160;
  float movementDistance = (distance ) ;
  int targetTicks = (movementDistance / wheelCircumference) * ticksPerRevolution * 0.8;
  encoderCount1 = 0;
  encoderCount2 = 0;
  digitalWrite(Motor1a, HIGH);  // Set IN1 HIGH
  digitalWrite(Motor1b, LOW);   // Set IN2 LOW
  analogWrite(ENA, speed);  // Set motor speed (0-255)
  digitalWrite(Motor2a, HIGH);  // Set IN1 HIGH
  digitalWrite(Motor2b, LOW);   // Set IN2 LOW
  analogWrite(ENB, speed);  // Set motor speed (0-255)
    // while (abs(encoderCount1) < targetTicks && abs(encoderCount2) < targetTicks) {

    //   while (Dist() < 50) { 
    //     rotate(90);
    //   }

    Serial.print("Encoder1St: ");
    Serial.print(encoderCount1);
    Serial.print(" Encoder2St: ");
    Serial.println(encoderCount2);
    if (abs(encoderCount1 - targetTicks) < 50){
        analogWrite(ENA, 100);
        analogWrite(ENB, 100);
    }
  // }
  stopMotors();
  encoderCount1 = 0;
  encoderCount2 = 0;
}

void stopMotors() {
  digitalWrite(Motor1a, LOW);
  digitalWrite(Motor1b, LOW);
  digitalWrite(Motor2a, LOW);
  digitalWrite(Motor2b, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}