#define DELAY 25
#define LEDPIN 14

// —————————————————————————  Motors
int motor1[] = {  
  2, 3};
int motor2[] = {  
  4, 5};
int motor3[] = {  
  6, 7};
int motor4[] = {  
  8, 9};
int motor5[] = {  
  10, 11};
int motor6[] = {  
  12, 13};

// ————————————————————————— Setup
void setup() {
  Serial.begin(9600);

  // Setup motors
  int i;
  for(i = 0; i < 2; i++){
    pinMode(motor1[i], OUTPUT);
    pinMode(motor2[i], OUTPUT);   
    pinMode(motor3[i], OUTPUT);
    pinMode(motor4[i], OUTPUT);
    pinMode(motor5[i], OUTPUT);
    pinMode(motor6[i], OUTPUT);    
  }
  pinMode(LEDPIN,OUTPUT);
  motorST(motor1);
  motorST(motor2);
  motorST(motor3);
  motorST(motor4);
  motorST(motor5);
  motorST(motor6);
  digitalWrite(LEDPIN,LOW);


}


void loop() {

  char cmd=0;


  if (Serial.available() > 0) 
  {

    cmd = Serial.read();

    switch (cmd) {

    case 'q':
      motorFW(motor1);
      break;
    case 'a':
      motorBW(motor1);
      break;   
    case 'w':
      motorFW(motor2);
      motorFW(motor3);      
      break;
    case 's':
      motorBW(motor2);
      motorBW(motor3);     
      break;     
    case 'e':
      motorFW(motor2);
      motorBW(motor3);      
      break;
    case 'd':
      motorBW(motor2);
      motorFW(motor3);     
      break;   
    case 'r':
      motorFW(motor4);
      motorFW(motor5);      
      break;
    case 'f':
      motorBW(motor4);
      motorBW(motor5);     
      break;     
    case 't':
      motorFW(motor4);
      motorBW(motor5);      
      break;
    case 'g':
      motorBW(motor4);
      motorFW(motor5);     
      break;      
    case 'y':
      motorFW(motor6);
      break;
    case 'h':
      motorBW(motor6);
      break;  
      case 'u':
      digitalWrite(LEDPIN,HIGH);
      break;
      case 'j':
      digitalWrite(LEDPIN,LOW);
      break;      
    }
    delay(DELAY);
    motorST(motor1);
    motorST(motor2);
    motorST(motor3);
    motorST(motor4);
    motorST(motor5);
    motorST(motor6);

  }



}



void motorFW(int motor[2]){
  digitalWrite(motor[0], HIGH);
  digitalWrite(motor[1], LOW);
}
void motorBW(int motor[2]){
  digitalWrite(motor[0], LOW);
  digitalWrite(motor[1], HIGH);
}
void motorST(int motor[2]){
  digitalWrite(motor[0], LOW);
  digitalWrite(motor[1], LOW);
}


