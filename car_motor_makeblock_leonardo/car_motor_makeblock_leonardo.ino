#define SPEEDGAP 10 
#define DELAYTIME 25
#define GAPRML 20
#define MINSPEED 20
#define MSGLEN 5
#define LASERPIN 2


#include <SPI.h>
#include <MirfMB.h>
#include <nRF24L01MB.h>
#include <MirfHardwareSpiDriverMB.h>
//#include <Makeblock.h>
//#include <Arduino.h>
//#include <SoftwareSerial.h>
//#include <Wire.h>

/*MeDCMotor motor3(M1);
MeDCMotor motor4(M2);

int SpeedR = 100;
int SpeedL = 255 - GAPRML;

void setup() {
 Serial.begin(9600);


  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();    ////   I've CHANGED CE AND CSN to 9 10
  Mirf.setRADDR((byte *)"serv1");
  Mirf.payload = MSGLEN*sizeof(byte);       
  Mirf.config();

pinMode(LASERPIN,OUTPUT);
}

void loop() {
 
  byte data[Mirf.payload];
  if(!Mirf.isSending() && Mirf.dataReady())
{
    Mirf.getData(data);

   for (int motor_right[] = {  4, 7;int i=0; i<MSGLEN+1;i++)   Serial.println( (int)data[i] ); 
 
 
   if ((char)data[1]==1)   motor3.run(SpeedR);
   else   motor3.run(-SpeedR); 
   if ((char)data[3]==1)   motor4.run(SpeedR);
   else   motor4.run(-SpeedR);  
  
   delay(DELAYTIME);
   motor3.stop();
   motor4.stop();
      
   if (data[4] ==1 )
     digitalWrite(LASERPIN,HIGH);
   else
     digitalWrite(LASERPIN,LOW);
 
    //motor3.run(SpeedR);
    //motor4.run(SpeedR);
    //delay(25);
    //motor3.stop();
    //motor4.stop();
 
  }
}*/

#define ENABLEMOTORL 9
#define ENABLEMOTORR 10
#define DT2STOP 100

unsigned long TimeNoComm = 0;

// —————————————————————————  Motors
int motor_left[] = {  6, 7 };
int motor_right[]d = {  5, 4};

// ————————————————————————— Setup
void setup() {
  Serial.begin(9600);

  // Setup motors
  int i;
  for(i = 0; i < 2; i++){
    pinMode(motor_left[i], OUTPUT);
    pinMode(motor_right[i], OUTPUT);
  }
  //pinMode(ENABLEMOTORL,OUTPUT);
  //pinMode(ENABLEMOTORR,OUTPUT);
  
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();    ////   I've CHANGED CE AND CSN to 9 10 ( 8 7 )
  Mirf.setRADDR((byte *)"serv1");
  Mirf.payload = MSGLEN*sizeof(byte);       
  Mirf.config();

TimeNoComm = millis();
/*
TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(CS12);
  OCR1A = 0;
  OCR1B = 0;
  */
}


void loop() {

  byte data[Mirf.payload];
  if(!Mirf.isSending() && Mirf.dataReady()){
    Mirf.getData(data);
  Serial.println(":");
    for (int i=0; i<MSGLEN;i++)   Serial.println( (int)data[i] ); 


    if (data[4] ==1 ) {
      if ((char)data[1]==1 &&  (char)data[3]==1 )  
      {
        drive_forward();
      }
      else if ((char)data[1]==0 &&  (char)data[3]==0 )  
      {
        drive_backward();
      }
      else if ((char)data[1]==1 &&  (char)data[3]==0 )  
      {
        turn_left();
      }
      else if ((char)data[1]==0 &&  (char)data[3]==1 )  
      {
        turn_right();
      }

      //analogWrite(ENABLEMOTORL, (char)data[0] );
      //analogWrite(ENABLEMOTORR, (char)data[2]  );      
    }
    else {
      //analogWrite(ENABLEMOTORL, 0);
      //analogWrite(ENABLEMOTORR, 0);
      motor_stop(); 

    } 
   delay(DELAYTIME);
   TimeNoComm = millis();
  }

if ( (millis() - TimeNoComm) > DT2STOP)
{
      //analogWrite(ENABLEMOTORL, 0);
      //analogWrite(ENABLEMOTORR, 0);
      motor_stop(); 
}


}


void motor_stop(){
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], LOW);

  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], LOW);
  delay(25);
}

void drive_forward(){
  digitalWrite(motor_left[0], HIGH);
  digitalWrite(motor_left[1], LOW);

  digitalWrite(motor_right[0], HIGH);
  digitalWrite(motor_right[1], LOW);
}

void drive_backward(){
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], HIGH);

  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], HIGH);
}

void turn_left(){
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], HIGH);

  digitalWrite(motor_right[0], HIGH);
  digitalWrite(motor_right[1], LOW);
}

void turn_right(){
  digitalWrite(motor_left[0], HIGH);
  digitalWrite(motor_left[1], LOW);

  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], HIGH);
}

  


