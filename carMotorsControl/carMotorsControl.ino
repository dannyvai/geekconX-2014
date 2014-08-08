#define MSGLEN 5

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

const char beginChar=255;

void setup(){
  Serial.begin(9600);
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"clie1");
  Mirf.payload = MSGLEN*sizeof(byte);
  Mirf.config();
  // Serial.println("Beginning ... "); 
}


void loop(){

  char sendingnum=0;
  char sendArray[MSGLEN];
  boolean  sendFlag=false;

  Mirf.setTADDR((byte *)"serv1");

  if (Serial.available() > 0) 
  {
    sendingnum = Serial.read();
    //      Serial.print(sendingnum);
    if (sendingnum==beginChar)
    {
      int i=0;
      while(!sendFlag)
  {
    if (Serial.available() > 0) 
  {
        sendArray[i]= Serial.read();
        i++;
  }
    if (i>MSGLEN)
      sendFlag=true;
    
    }
    }
  }
  if (sendFlag)
  {
    Mirf.send((byte *)sendArray);
   // for (int i=0; i<3;i++)   Serial.print( sendArray[i] ); 
  //  Serial.println( sendArray[3]);
    while(Mirf.isSending()){
    }
  }
}


