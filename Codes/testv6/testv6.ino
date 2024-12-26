volatile long int encoderPosA = 0;
volatile long int encoderPosB = 0;
volatile long int refEncoderPosA = 0;
volatile long int refEncoderPosB = 0;

#define encoderOne 2
#define encoderTwo 3
#define Motor1a 8
#define Motor1b 9
#define Motor2a 4
#define Motor2b 7
const int ENA = 5; 
const int ENB = 6;

// Function to initialize motor pins
void setup() {
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoderOne), incCountA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderTwo), incCountB, RISING);

  pinMode(Motor1a, OUTPUT);
  pinMode(Motor1b, OUTPUT);
  pinMode(Motor2a, OUTPUT);
  pinMode(Motor2b, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Start with the motor off
  stopMotor();
}

void incCountA(){
  encoderPosA++;
}

void incCountB(){
  encoderPosB++;
}

// Function to move the motor forward
void forward(int speed) {
  digitalWrite(Motor1a, HIGH);  // Set IN1 HIGH
  digitalWrite(Motor1b, LOW);   // Set IN2 LOW
  analogWrite(ENA, speed);  // Set motor speed (0-255)
  digitalWrite(Motor2a, HIGH);  // Set IN1 HIGH
  digitalWrite(Motor2b, LOW);   // Set IN2 LOW
  analogWrite(ENB, speed);  // Set motor speed (0-255)
}

void rotate(int angle) {
  int speed = 110;
  int const1 = 1;
  refEncoderPosA = encoderPosA;
  refEncoderPosB = encoderPosB;
  while ((encoderPosA - refEncoderPosA + encoderPosB - refEncoderPosB)<= 440){
    digitalWrite(Motor1a, HIGH);  // Set IN1 HIGH
    digitalWrite(Motor1b, LOW);   // Set IN2 LOW
    analogWrite(ENA, speed);  // Set motor speed (0-255)
    digitalWrite(Motor2a, LOW);  // Set IN1 HIGH
    digitalWrite(Motor2b, HIGH);   // Set IN2 LOW
    analogWrite(ENB, speed);  // Set motor speed (0-255)
  }
  // delay(const1 * angle);
}

// Function to stop the motor
void stopMotor() {
  digitalWrite(Motor1a, LOW);  // Set IN1 HIGH
  digitalWrite(Motor1b, LOW);   // Set IN2 LOW
  analogWrite(ENA, 0);      // Set speed to 0
  digitalWrite(Motor2a, LOW);  // Set IN1 HIGH
  digitalWrite(Motor2b, LOW);   // Set IN2 LOW
  analogWrite(ENB, 0);
}

// Main loop
void loop() {

  forward(180);
  delay(800);
  rotate(180);
  stopMotor();
  delay(1000);

}
