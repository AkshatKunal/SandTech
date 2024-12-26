volatile long int encoderPos = 0;
int motor_value = 255;
float startTime;
volatile float currentTime;

#define encoderOne 2
#define Motor1a 11
#define Motor1b 12
#define Motor2a 13
#define Motor2b 7


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoderOne), blink, RISING);

  pinMode(Motor1a, OUTPUT);
  pinMode(Motor1b, OUTPUT);
  pinMode(Motor2a, OUTPUT);
  pinMode(Motor2b, OUTPUT);

  pinMode(encoderOne, INPUT);

  startTime = millis();

}

void blink(){
  encoderPos++;
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
}
