int array1[6] = {2,3,4,7};

void setup() {
  // put your setup code here, to run once:
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(2,HIGH);

  for (int i ; i < 6; i++){
    int a = array1[i];
    if (a%2 == 0){
      digitalWrite(a,HIGH);
    }else{
      digitalWrite(a,HIGH);
    }
  }
}
