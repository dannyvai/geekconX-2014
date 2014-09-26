#include "Encoder_Polling_V2.h"

#define DELAY 25
#define LEDPIN 14
 #define PUMPPIN 15
 

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
 
 encoder_begin();  // Start the library
  attach_encoder(0, 20, 21);  // Attach an encoder to pins A and B
  attach_encoder(1, 18, 19);  // Attach another encoder to pins C and D

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
  pinMode(PUMPPIN,OUTPUT);
  
  motorST(motor1);
  motorST(motor2);
  motorST(motor3);
  motorST(motor4);
  motorST(motor5);
  motorST(motor6);
  digitalWrite(LEDPIN,LOW);
  digitalWrite(PUMPPIN,HIGH);

}

 int counter2=0;
 int counter3=0;
 int lastcounter2=0;
 int lastcounter3=0;
void loop() {
char cmd;
  int dir_2 = encoder_data(0);  // First encoder
  int dir_3 = encoder_data(1);  // Second
 
  if(dir_2 != 0)       // If its forward...
  {
    counter2+= dir_2;       // Increment the counter
    //Serial.println(counter2);
  }
  if(dir_3 != 0)       // If its forward...
  {
    counter3+= dir_3;       // Increment the counter
    //Serial.println(counter3);
  }
  
  
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
   case 'i':
      digitalWrite(PUMPPIN,LOW);
      break;
      case 'k':
      digitalWrite(PUMPPIN,HIGH);
      break;     
    }
    delay(DELAY);

    motorST(motor1);
    motorST(motor2);
    motorST(motor3);
    motorST(motor4);
    motorST(motor5);
    motorST(motor6);
    lastcounter2 =counter2;
    lastcounter3 =counter3;

  }
  else
  {
    if (lastcounter2>counter2)
    {
      motorFW(motor2);
    }
      else
      {
      motorST(motor2);
    }
    if (lastcounter3<counter3)
    {
      motorBW(motor3);
    }
      else
      {
      motorST(motor3);
    }
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


