#define SPEEDGAP 10 
#define DELAYTIME 25
#define GAPRML 20
#define MINSPEED 20
#define MSGLEN 5
#define LASERPIN 2


#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>




int SpeedR = 255;
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
  if(!Mirf.isSending() && Mirf.dataReady()){
    Mirf.getData(data);

   for (int i=0; i<MSGLEN+1;i++)   Serial.println( (int)data[i] ); 
 
  }
}

  


