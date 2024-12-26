#define ENCODER_PIN_A 2  // Channel A for encoder
#define ENCODER_PIN_B 3  // Channel B (optional for direction sensing)
#define MOTOR_PIN1 9     // Motor driver pin 1
#define MOTOR_PIN2 10    // Motor driver pin 2
#define MOTOR_PWM 11     // PWM speed control pin

volatile unsigned long pulseCount = 0; // Counts encoder pulses
unsigned long lastTime = 0;
const int pulsesPerRevolution = 20; // Adjust this based on your encoder's specification

int motorSpeed = 150; // Set initial motor speed (0-255)

void countPulses() {
  pulseCount++;
}

void setup() {
  // Motor and encoder setup
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), countPulses, RISING);

  Serial.begin(9600);
  Serial.println("Motor speed set to: 150");
}

void loop() {
  // Drive the motor at the predefined speed
  digitalWrite(MOTOR_PIN1, HIGH);  // Set motor direction
  digitalWrite(MOTOR_PIN2, LOW);
  analogWrite(MOTOR_PWM, motorSpeed);

  // Measure RPM every second
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 1000) { // Calculate RPM every 1 second
    float rpm = (pulseCount / (float)pulsesPerRevolution) * 60.0;
    pulseCount = 0; // Reset pulse count
    lastTime = currentTime;

    // Display RPM
    Serial.print("RPM: ");

    Serial.println(rpm);
  }
}
