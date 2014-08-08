
 /*
  if(keyPressed) {
       port.write (key);
}
*/
import processing.serial.*;

Serial port;
String portname = "COM6";  
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
void setup() 
{
    port = new Serial(this, portname, baudrate);
  println(port);
  size(1720, 880);
  bx = width/2.0;
  by = height/2.0;
  rectMode(RADIUS);  
  ellipseMode(RADIUS);
}

void draw() 
{ 
  background(0);
  
  // Test if the cursor is over the box 
  if (mouseX > bx-boxSize && mouseX < bx+boxSize && 
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
  }
   ellipse(width/2.0, height/2.0, 20, 20); 
  // Draw the box
  rect(bx, by, boxSize, boxSize);

float RELbx = (bx-width/2.0)*2;
float RELby = (by-height/2.0)*2;
float ROTbx = cos(3*PI/4)*RELbx - sin(3*PI/4)*RELby;
float ROTby = sin(3*PI/4)*RELbx + cos(3*PI/4)*RELby;
  char xPower = (char)min(maxSpeed,(char)abs(ROTbx));
  char yPower = (char)min(maxSpeed,(char)abs(ROTby));  
  char xDir = ((ROTbx) > 0 ) ? (char)1 : (char)0;
  char yDir = ((ROTby) > 0 ) ? (char)1 : (char)0;
  if ((xPower>20 || yPower > 20) && locked ) {
  port.write(255);
  port.write(xPower);
  port.write(xDir);
  port.write(yPower);
  port.write(yDir);  
  port.write(pinFlag ? (char)1 : (char)0);  
   while (port.available() > 0) {
    int inByte = port.read();
    println(inByte);
  }
  }
  println("X:" + (int)xPower + "(" + (int)xDir + ")" + " Y:" + (int)yPower + "(" + (int)yDir + ")"  );
}

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


