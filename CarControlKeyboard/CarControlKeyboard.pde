
 /*
  if(keyPressed) {
       port.write (key);
}
*/
import processing.serial.*;

Serial port;
String portname = "/dev/ttyACM0";  
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
final char maxSpeed=254;
int speedL = 128;
int speedR = 254;
 char xDir = 0;
  char yDir = 0;
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
   default: 
    locked=false;
  }
  }

  // Test if the cursor is over the box 
/*  if (mouseX > bx-boxSize && mouseX < bx+boxSize && 
      mouseY > by-boxSize && mouseY < by+boxSize) {
    overBox = true;  
    if(!locked) { 
      stroke(255); 
      fill(153);
    } 
  } else {
    stroke(153);
    fill(153);
    overBox = false;
  }*/
   ellipse(width/2.0, height/2.0, 20, 20); 
  // Draw the box
  float RELbx =  width/2.0 - ( ((float)xDir - 0.5)*2 )*(float)speedL ;
float RELby =   height/2.0 -( ((float)yDir - 0.5)*2 )*(float)speedR;
  rect(RELbx , RELby , boxSize, boxSize);

  char xPower = (char)speedL;
  char yPower = (char)speedR;
  if ((xPower>20 || yPower > 20) && locked ) {
  port.write(255);
  port.write(xPower);
  port.write(xDir);
  port.write(yPower);
  port.write(yDir);  
  port.write(1);  
//   while (port.available() > 0) {
//    int inByte = port.read();
//    println(inByte);
//  }
  }
  println("X:" + (int)xPower + "(" + (int)xDir + ")" + " Y:" + (int)yPower + "(" + (int)yDir + ")"  );
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
