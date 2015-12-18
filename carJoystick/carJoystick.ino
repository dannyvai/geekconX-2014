#define MSGLEN 5
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

void setup()  
{
  //int ind;
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


  Serial.println("Goodnight moon!");
  //  for (ind=2; ind<=5; ind++)
  //  {
  //  pinMode(ind, OUTPUT);
  //    digitalWrite(ind,LOW);
  //  }

  Mirf.spi = &MirfHardwareSpi;

  /*
   * Setup pins / SPI.
   */

  Mirf.init();

  /*
   * Configure reciving address.
   */

  Mirf.setRADDR((byte *)"clie1");

  Mirf.payload = sizeof(char)*MSGLEN;

  /*
   * Write channel and payload config then power up reciver.
   */

  Mirf.config();
 // Mirf.setTADDR((byte *)"serv1");
}
boolean gyrokey = false;

void loop() // run over and over
{

  byte data[5];
  byte getData[5];
  byte data2send[5];
  byte data2send2[5];
  //delay(70);
  byte ind;
  boolean gyroFlag;
  
  char ch;

  for (int i=0; i<MSGLEN; i++) data[i]=0;
  gyroFlag=false;
  data2send2[1]=0;
  if(!Mirf.isSending() && Mirf.dataReady()){
    Serial.println("try to getData");
    Mirf.getData((byte *)getData);
    Serial.println("OK");
    getData[4] = 1-getData[4];
    gyro2data(getData,data);
    Serial.println("after converting");
    gyroFlag=true;
    if (getData[4] == 1 && !gyrokey)
    {
      gyrokey = true;
      data2send2[0] = 0;
      data2send2[1] = 6;
      data2send2[2] = (byte)(micros() % 120);
      data2send2[3] = (byte)((micros()/10) % 120);
      data2send2[4] = 0;//(millis()/100) % 120;
      
    }
    if (getData[4] == 0 && gyrokey)
    {
      gyrokey = false;
      data2send2[0] = 0;
      data2send2[1] = 6;
      data2send2[2] = 0;
      data2send2[3] = 0;
      data2send2[4] = 0;
      
    }
    
//    for (int i=0; i<MSGLEN; i++){
//      Serial.print((int)getData[i]); 
//      Serial.print(",");
//    }
//    Serial.println("");
  }
Serial.println("after getData");
//    joystick2data(getData);
//    if (getData[1] >0 || getData[3]>0)
//    {
//      for (int i=0; i<MSGLEN; i++)
//        data[i]=getData[i];
//    }
//    else
//    {
//      if (!gyroFlag)
//      {
//        data[1]=0;
//        data[3]=0;
//      }
//    }

  //  Serial.print((unsigned byte)data[0]);
  //  Serial.print(",");
  //  Serial.print((unsigned byte)data[1]);
  //  Serial.print(",");
  //  Serial.print((unsigned byte)data[2]);
  //  Serial.print(",");
  //  Serial.print((unsigned byte)data[3]);
  //  Serial.print(",");
  //  Serial.println((unsigned byte)data[4]);

  data2send[4]= 5;
  if (data[1] ==1 && data[3] ==1)
  {
    data2send[1] = 1;
    data2send[2] = data[2];
    data2send[3] = data[4];
  }
  if (data[1] ==2 && data[3] ==2)
  {
    data2send[1] = 2;
    data2send[2] = data[2];
    data2send[3] = data[4];
  }
  if (data[1] ==1 && data[3] ==2)
  {
     data2send[4]= 2;
    data2send[1] = 4;
    data2send[2] = data[2]-30;
    data2send[3] = data[4]-30;
  }
  if (data[1] ==2 && data[3] ==1)
  {
     data2send[4]= 2;
    data2send[1] = 3;
    data2send[2] = data[2]-30;
    data2send[3] = data[4]-30;
  }


//   if (Serial.available())
//   {
//     ch = Serial.read();
//     switch (ch)
//     {
//      case 'r':
//         data2send[0] = 0;
//      data2send[1] = 6;
//      data2send[2] = 250;
//      data2send[3] = 0;
//      data2send[4] = 0;
//      Mirf.send((byte *)data2send);
//      while(Mirf.isSending());
//     break;
//     case 'b':
//        data2send[0] = 0;
//      data2send[1] = 6;
//      data2send[2] = 0;
//      data2send[3] = 0;
//      data2send[4] = 0;
//      Mirf.send((byte *)data2send);
//      while(Mirf.isSending());
//     break;
//     }
//   }
//
//

 Mirf.setTADDR((byte *)"serv1");

  if (data[1]>0 ||data[3]>0 || data2send2[1]==6)
  {
    //for (ind =0; ind<5; ind++)
    {
      data2send[0] = 0;
      if (data2send2[1]==6)
      {
        for (ind =0; ind<5; ind++)
        data2send[ind] = data2send2[ind];
      }
      Serial.println("try to send");
      Mirf.send((byte *)data2send);
    Serial.println("OK");
    //  while(Mirf.isSending());
      Serial.print("Sending: ");
      for (int i=0; i<MSGLEN; i++){
        Serial.print(data2send[i]); 
        Serial.print(",");
      }
      Serial.println("");
      
    }
  }
}


void joystick2data(byte *data)
{
  float m1, m2, X,Y, XpY, sXpY, XmY;

  X = (float)analogRead(0)/512.0 - 1.0; //*500/300-250;
  Y = -1.0*(float)analogRead(1)/512.0 + 1.0; //*500/300-250;
  XpY = X*X+Y*Y;
  sXpY = sqrt(XpY);
  XmY = X*X-Y*Y;
  m1 = 0;
  m2 = 0;

  if (X >= 0 && Y >= 0)
  {
    m1 = sXpY;
    m2 = -XmY/sXpY;
  }
  if (X < 0 && Y > 0)
  {
    m2 = sXpY;
    m1 = -XmY/sXpY;
  }
  if (X <= 0 && Y <= 0)
  {
    m1 = -sXpY;
    m2 = XmY/sXpY;
  }
  if (X > 0 && Y < 0)
  {
    m2 = -sXpY;
    m1 = XmY/sXpY;
  }

  if (abs(X)<0.2 && abs(Y)<0.2)
  {
    data[0]=101;
    data[1]=0;
    data[3]=0;
    return;
  }

  m1 = max(-1,min(m1,1));
  m2 = max(-1,min(m2,1));

  data[0]=101;
  if (m2 > 0)
    data[1]=1;
  else
    data[1]=2;
  if (m1 > 0)
    data[3]=1;
  else
    data[3]=2;

  data[2]=(char)abs(round(m2*255));//(byte)(round(abs(X)*255));
  data[4]=(char)abs(round(m1*255));//(byte)(round(abs(Y)*255));
  //  fill(150);
  // rect(0, 0, data[2], data[4]); 

}

void gyro2data(byte *dataIn, byte *data)
{
  float m1, m2, X,Y, XpY, sXpY, XmY;

  if (dataIn[0]<=90/(dataIn[4]+1))
    X = (float)dataIn[0]/(float)(90/(dataIn[4]+1));
  else
    X = 1.0;
  if (dataIn[1] == 1)
    X = -1.0*X;
  if (dataIn[2]<=90/(dataIn[4]+1))
    Y = (float)dataIn[2]/(float)(90/(dataIn[4]+1));
  else
    Y = 1.0;
  if (dataIn[3] == 1)
    Y = -1.0*Y;

  XpY = X*X+Y*Y;
  sXpY = sqrt(XpY);
  XmY = X*X-Y*Y;
  m1 = 0;
  m2 = 0;

  if (X >= 0 && Y >= 0)
  {
    m1 = sXpY;
    m2 = -XmY/sXpY;
  }
  if (X < 0 && Y > 0)
  {
    m2 = sXpY;
    m1 = -XmY/sXpY;
  }
  if (X <= 0 && Y <= 0)
  {
    m1 = -sXpY;
    m2 = XmY/sXpY;
  }
  if (X > 0 && Y < 0)
  {
    m2 = -sXpY;
    m1 = XmY/sXpY;
  }

  if (abs(X)<0.2 && abs(Y)<0.2)
  {
    data[0]=101;
    data[1]=0;
    data[3]=0;
    return;
  }

  m1 = max(-1,min(m1,1));
  m2 = max(-1,min(m2,1));

  data[0]=101;
  if (m2 > 0)
    data[1]=1;
  else
    data[1]=2;
  if (m1 > 0)
    data[3]=1;
  else
    data[3]=2;

  data[2]=(char)abs(round(m2*255));//(byte)(round(abs(X)*255));
  data[4]=(char)abs(round(m1*255));//(byte)(round(abs(Y)*255));
  //  fill(150);
  // rect(0, 0, data[2], data[4]); 

}



