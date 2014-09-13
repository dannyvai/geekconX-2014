
 /*
  if(keyPressed) {
       port.write (key);
}
*/
import processing.serial.*;
byte MYID=3;
boolean allGo=false;

Serial port;
String portname = "COM6";//"/dev/ttyACM0";  
int baudrate = 9600;
int value = 0;  
 
float bx;
float by;
int boxSize = 20;
boolean overBox = false;
boolean locked = false;
boolean pinFlag = false;
float xOffset = 0.0; 
float yOffset = 0.0; 
float radius = 0.0;
final byte maxSpeed=(byte)254;
int speedL = 230;
int speedR = 254;
 byte xDir = 0;
  byte yDir = 0;
int iii1=1;
int iii2=0;
int iii3=0;
void setup() 
{
    port = new Serial(this, portname, baudrate);
  println(port);
  size(512, 512);
  bx = width/2.0;
  by = height/2.0;
  rectMode(RADIUS);  
  ellipseMode(RADIUS);
}

void draw() 
{ 
  background(0);
  boolean stop=false;
  boolean leds=false;
  
  if(locked) { 
  switch (key) {
   case 'a':
    speedL = min(254,speedL + 5);
    break;
   case 'z':
    speedL = max(0,speedL - 5);
    break;
   case 's':
    speedR = min(254,speedR + 5);
    break;
   case 'x':
    speedR = max(0,speedR - 5);
    break;    
   case 50:
   xDir = 0;
   yDir = 0;
    break;
   case 52:
   xDir = 1;
   yDir = 0;   
    break;
   case 54:
   xDir = 0;
   yDir = 1;      
    break;
   case 56:
   xDir = 1;
   yDir = 1;      
    break;  
   case 'q':
    stop=true;     
    break; 
   case 'w':
    leds=true; 
if (iii1==1) {iii1=0;iii2=1;}
else if (iii2==1) {iii2=0;iii3=1;}
else if (iii3==1) {iii3=0;iii1=1;}
    break;     
   case '1':
    MYID=1; 
    allGo = false;       
    break;   
   case '3':
    MYID=2; 
    allGo = false;       
    break;  
    case '5':
    MYID=3;   
    allGo = false;       
    break;  
    case '7':
    allGo = false;       
    MYID=4;     
    break;
    case '9':
    MYID=5; 
    allGo = false;      
    break;     
    case '0':
    allGo = true;     
    break;       
   default: 
    locked=false;
  }
  }
   ellipse(width/2.0, height/2.0, 20, 20); 
  // Draw the box
  float RELbx =  width/2.0 - ( ((float)xDir - 0.5)*2 )*(float)speedL ;
float RELby =   height/2.0 -( ((float)yDir - 0.5)*2 )*(float)speedR;
  rect(RELbx , RELby , boxSize, boxSize);

  byte xPower = (byte)speedL;
  byte yPower = (byte)speedR;
  if (((char)xPower>20 || (char)yPower > 20) && locked ) {
    if (allGo) MYID=5;
    do {
  port.write(255);   port.write(254);   port.write(100);
  port.write(MYID);
  if (stop)   {
   port.write(5);
   port.write(0);
   port.write(0);
   port.write(0);  }
  else if (leds)   {
   port.write(6);    port.write(100*iii1);     port.write(100*iii2);     port.write(100*iii3);   }
  else   {
  switch (2*xDir + yDir)
  {
    case 0:
    port.write(2);
    break;
    case 1: 
    port.write(4);
    break;
    case 2: 
    port.write(3);
    break;
    case 3: 
    port.write(1);
    break;    
  }
  port.write(xPower);  port.write(yPower);  port.write(1);  
  }
  if (allGo) MYID--;
    } while ( allGo && MYID>0 );
  //          print(":");
  //     for (int iii=0;iii<6;iii++) {
  // while (port.available() > 0) { 
  // char inByte = (char)port.read();
  //  println((int)inByte);
  // }
//  }
  println("X:" + (int)xPower + "(" + (int)xDir + ")" + " Y:" + (int)yPower + "(" + (int)yDir + ")"  );
  }
}



void keyPressed()
{
 locked = true; 
 // println((int)key);
}

void keyReleased()
{
 locked = false; 
}

/*
void mousePressed() {
  if(overBox) { 
    locked = true; 
    fill(255, 255, 255);
  } else {
    locked = false;
  }
  xOffset = mouseX-bx; 
  yOffset = mouseY-by; 
  
    if (mouseButton == LEFT)
  pinFlag=true;
  if (mouseButton == RIGHT)
  pinFlag=false;

}

void mouseDragged() {
  if(locked) {
    bx = mouseX-xOffset; 
    by = mouseY-yOffset; 
  }
}

void mouseReleased() {
  locked = false;
}

*/
