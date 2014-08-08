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
#include <Makeblock.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

MeDCMotor motor3(M1);
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

   for (int i=0; i<MSGLEN+1;i++)   Serial.println( (int)data[i] ); 
 
 
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
 /*
    motor3.run(SpeedR);
    motor4.run(SpeedR);
    delay(25);
    motor3.stop();
    motor4.stop();
 */
  }
}

  


