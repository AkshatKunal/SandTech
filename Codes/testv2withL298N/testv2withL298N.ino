volatile long int encoderPos = 0;
int motor_value = 255;
float startTime;
volatile float currentTime;

#define encoderOne 2
#define Motor1a 11
#define Motor1b 12
#define Motor2a 13
#define Motor2b 7
const int ENA = 10; 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoderOne), blink, RISING);

  pinMode(Motor1a, OUTPUT);
  pinMode(Motor1b, OUTPUT);
  pinMode(Motor2a, OUTPUT);
  pinMode(Motor2b, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(encoderOne, INPUT);

  startTime = millis();

}

void blink(){
  encoderPos++;
}

void forward(int speed) {
  digitalWrite(Motor1a, HIGH);  // Set IN1 HIGH
  digitalWrite(Motor1b, LOW);   // Set IN2 LOW
  analogWrite(ENA, speed);  // Set motor speed (0-255)
}

void stopMotor() {
  digitalWrite(Motor1a, LOW);
  digitalWrite(Motor1b, LOW);
  analogWrite(ENA, 0);      // Set speed to 0
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();

  if (currentTime - startTime >= 1000){
    Serial.print("Position: ");
    Serial.println(encoderPos);
    encoderPos = 0;
    startTime = currentTime;
  }

  forward(200);    // Move forward at 80% speed
  delay(2000);     // Run for 2 seconds

  stopMotor();     // Stop the motor
  delay(2000);     // Pause for 2 seconds
}
