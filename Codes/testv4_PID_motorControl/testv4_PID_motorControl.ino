volatile long int encoderPos = 0;
volatile long int refEncoderPos = 0;
int motor_value = 255;
float startTime;
volatile float currentTime;
unsigned long lastTime = 0;

#define encoderOne 2
#define encodertwo 3
#define Motor1a 11
#define Motor1b 12
#define Motor2a 13
#define Motor2b 7
const int ENA = 10; 
const int ENB = 9;

double dt;
double integral, previous, output = 0;
double kp, ki, kd;
float speedError = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoderOne), incCount, RISING);

  pinMode(Motor1a, OUTPUT);
  pinMode(Motor1b, OUTPUT);
  pinMode(Motor2a, OUTPUT);
  pinMode(Motor2b, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(encoderOne, INPUT);

  startTime = millis();

}

void incCount(){
  encoderPos++;
}

double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}

void forward(int time, int speed) {
  digitalWrite(Motor1a, HIGH);  // Set IN1 HIGH
  digitalWrite(Motor1b, LOW);   // Set IN2 LOW
  analogWrite(ENA, speed);  // Set motor speed (0-255)
  digitalWrite(Motor2a, HIGH);  // Set IN1 HIGH
  digitalWrite(Motor2b, LOW);   // Set IN2 LOW
  analogWrite(ENB, speed);  // Set motor speed (0-255)

  float actualSpeed = (speed * 550)/255;

  unsigned long currentTimeDatum = millis();
  unsigned long lastCurrentTime = currentTime;
  unsigned long currentCurrentTime = millis();
  int currentEncoderPos = encoderPos;
  float setpointSpeed = actualSpeed;

  while (currentCurrentTime - currentTimeDatum < time){
    currentEncoderPos = encoderPos;
    currentCurrentTime = millis();
    dt = currentCurrentTime - lastCurrentTime;

    setpointSpeed = ((currentEncoderPos - refEncoderPos)* 60 / 260)/ (currentCurrentTime - lastCurrentTime);//Take from encoder

    double error = setpointSpeed - actualSpeed;
    speedError = pid(error);

    speed+= speedError;

    analogWrite(ENA, speed);  // Set motor speed (0-255)
    analogWrite(ENB, speed);  // Set motor speed (0-255)

    actualSpeed = (speed * 550)/255;
    lastCurrentTime = currentCurrentTime;
    refEncoderPos = currentEncoderPos;
  }
}

void stopMotor(int time, int speed) {
  digitalWrite(Motor1a, LOW);
  digitalWrite(Motor1b, LOW);
  analogWrite(ENA, 0);      // Set speed to 0
  digitalWrite(Motor2a, LOW);  // Set IN1 HIGH
  digitalWrite(Motor2b, LOW);   // Set IN2 LOW
  analogWrite(ENB, 0);  // Set motor speed (0-255)
  delay(time);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();
  refEncoderPos = encoderPos;
  forward(2000,255);
  delay(1000);
  encoderPos = 0;

}
