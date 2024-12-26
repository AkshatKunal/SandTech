// Define motor control pins
const int IN1 = 11;     // Motor direction control pin 1
const int IN2 = 12;     // Motor direction control pin 2
const int ENA = 10;    // Motor speed control (PWM)

// Function to initialize motor pins
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Start with the motor off
  stopMotor();
}

// Function to move the motor forward
void forward(int speed) {
  digitalWrite(IN1, HIGH);  // Set IN1 HIGH
  digitalWrite(IN2, LOW);   // Set IN2 LOW
  analogWrite(ENA, speed);  // Set motor speed (0-255)
}

// Function to move the motor backward
void backward(int speed) {
  digitalWrite(IN1, LOW);   // Set IN1 LOW
  digitalWrite(IN2, HIGH);  // Set IN2 HIGH
  analogWrite(ENA, speed);  // Set motor speed (0-255)
}

// Function to stop the motor
void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);      // Set speed to 0
}

// Main loop
void loop() {
  forward(200);    // Move forward at 80% speed
  delay(2000);     // Run for 2 seconds

  backward(150);   // Move backward at 60% speed
  delay(2000);     // Run for 2 seconds

  // stopMotor();     // Stop the motor
  // delay(2000);     // Pause for 2 seconds
}
