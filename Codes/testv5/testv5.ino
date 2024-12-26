/*README 
Before executing program:
Check motor 1 and 2 to left and right and write it:
Constants kp,kd,ki:
Check pin connections and note them:
Calibrate the angles and constants A and B:
Check dir for 1 and 2 which directions:
*/
volatile long int encoderPosA = 0;
volatile long int encoderPosB = 0;
volatile long int refEncoderPosA = 0;
volatile long int refEncoderPosB = 0;
int motor_value = 255;
float startTime;
volatile float currentTime;
unsigned long lastTime = 0;

#define encoderOne 2
#define encoderTwo 3
#define Motor1a 11
#define Motor1b 12
#define Motor2a 13
#define Motor2b 7
const int ENA = 10; 
const int ENB = 9;

double dt;
double integral, previous, output = 0;
double kp, ki, kd;
float speedErrorA = 0;
float speedErrorB = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoderOne), incCountA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderTwo), incCountB, RISING);

  pinMode(Motor1a, OUTPUT);
  pinMode(Motor1b, OUTPUT);
  pinMode(Motor2a, OUTPUT);
  pinMode(Motor2b, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(encoderOne, INPUT);
  pinMode(encoderTwo, INPUT);

  startTime = millis();

}

void incCountA(){
  encoderPosA++;
}

void incCountB(){
  encoderPosB++;
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

void rotate(float angleDiff){
  //angle diff between -180 and 180
  //Fint the sensitivity and tune to turn
  float constantA = 1;
  float constantB = 1;
  if (angleDiff>= 90){
    move(constantA * angleDiff,  240,  240,3);
  }else if(angleDiff <= -90){
    move(constantA * angleDiff,  240, 240,4);
  }else{
    float speedDiff = constantB * abs(angleDiff);
  }
}

void move(int time, float speedA, float speedB, int dir) {

  if (dir == 1){
    digitalWrite(Motor1a, HIGH);  // Set IN1 HIGH
    digitalWrite(Motor1b, LOW);   // Set IN2 LOW
    analogWrite(ENA, speedA);  // Set motor speed (0-255)
    digitalWrite(Motor2a, HIGH);  // Set IN1 HIGH
    digitalWrite(Motor2b, LOW);   // Set IN2 LOW
    analogWrite(ENB, speedB);  // Set motor speed (0-255)
  }else if (dir == 2){
    digitalWrite(Motor1a, LOW);  // Set IN1 HIGH
    digitalWrite(Motor1b, HIGH);   // Set IN2 LOW
    analogWrite(ENA, speedA);  // Set motor speed (0-255)
    digitalWrite(Motor2a, LOW);  // Set IN1 HIGH
    digitalWrite(Motor2b, HIGH);   // Set IN2 LOW
    analogWrite(ENB, speedB);  // Set motor speed (0-255)
  }else if (dir == 3){
    digitalWrite(Motor1a, HIGH);  // Set IN1 HIGH
    digitalWrite(Motor1b, LOW);   // Set IN2 LOW
    analogWrite(ENA, speedA);  // Set motor speed (0-255)
    digitalWrite(Motor2a, LOW);  // Set IN1 HIGH
    digitalWrite(Motor2b, HIGH);   // Set IN2 LOW
    analogWrite(ENB, speedB);  // Set motor speed (0-255)
  }else if (dir == 4){
    digitalWrite(Motor1a, LOW);  // Set IN1 HIGH
    digitalWrite(Motor1b, HIGH);   // Set IN2 LOW
    analogWrite(ENA, speedA);  // Set motor speed (0-255)
    digitalWrite(Motor2a, HIGH);  // Set IN1 HIGH
    digitalWrite(Motor2b, LOW);   // Set IN2 LOW
    analogWrite(ENB, speedB);  // Set motor speed (0-255)
  }

  float actualSpeedA = (speedA * 550)/255;
  float actualSpeedB = (speedB * 550)/255;
  Serial.print(actualSpeedA);
  Serial.print("1");
  Serial.println(actualSpeedB);

  unsigned long currentTimeDatum = millis();
  unsigned long lastCurrentTime = currentTime;
  unsigned long currentCurrentTime = millis();
  int currentEncoderPosA = encoderPosA;
  float setpointSpeedA = actualSpeedA;
  int currentEncoderPosB = encoderPosB;
  float setpointSpeedB = actualSpeedB;

  while (currentCurrentTime - currentTimeDatum < time){
    currentEncoderPosA = encoderPosA;
    currentEncoderPosB = encoderPosB;
    currentCurrentTime = millis();
    dt = currentCurrentTime - lastCurrentTime;

    setpointSpeedA = ((currentEncoderPosA - refEncoderPosA)* 60 / 260)/ (currentCurrentTime - lastCurrentTime);//Take from encoder
    setpointSpeedB = ((currentEncoderPosB - refEncoderPosB)* 60 / 260)/ (currentCurrentTime - lastCurrentTime);

    double errorA = setpointSpeedA - actualSpeedA;
    speedErrorA = pid(errorA);
    Serial.print(errorA);
    Serial.print("3");
    double errorB = setpointSpeedB - actualSpeedB;
    speedErrorB = pid(errorB);
    Serial.println(errorB);

    actualSpeedA += speedErrorA;
    actualSpeedB += speedErrorB;

    Serial.print(actualSpeedA);
    Serial.print("2");
    Serial.println(actualSpeedB);

    speedA = (actualSpeedA * 255)/550;
    speedB = (actualSpeedB * 255)/550;

    analogWrite(ENA, speedA);  // Set motor speed (0-255)
    analogWrite(ENB, speedB);  // Set motor speed (0-255)

    actualSpeedA = (speedA * 550)/255;
    actualSpeedB = (speedB * 550)/255;

    delay(500);

    lastCurrentTime = currentCurrentTime;
    refEncoderPosA = currentEncoderPosA;
    refEncoderPosB = currentEncoderPosB;
    //
    
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
  refEncoderPosA = encoderPosA;
  refEncoderPosB = encoderPosB;
  move(2000,  250,  250,1);
  // rotate(180);
  stopMotor(3000,3);
  encoderPosA = 0;
  encoderPosB = 0;
}
