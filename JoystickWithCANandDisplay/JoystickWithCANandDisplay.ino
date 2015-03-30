#include <AltSoftSerial.h>

AltSoftSerial dispSerial;
/*
  DigitalReadSerial
 Reads a digital input on pin 2, prints the result to the serial monitor 
 
 This example code is in the public domain.
 */


int rightButton = 14;
int downButton = 15;
int pushButton = 16;
int leftButton = 2;
int upButton = 3;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at bits per second:
  Serial.begin(115200);
  
  dispSerial.begin(9600);
  dispSerial.print("Hello World");
  
  // make the pushbutton's pin an input:
  pinMode(rightButton, INPUT);
  pinMode(leftButton, INPUT);
  pinMode(downButton, INPUT);
  pinMode(pushButton, INPUT);
  pinMode(upButton, INPUT);
  
  pinMode(A3, OUTPUT); //Enable the switches.
  digitalWrite(A3,HIGH);
  
  dispSerial.write(254); // move cursor to beginning of first line
  dispSerial.write(128);

  dispSerial.write("                "); // clear display
  dispSerial.write("                ");
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  int rightButtonState = digitalRead(rightButton);
  int leftButtonState = digitalRead(leftButton);
  int downButtonState = digitalRead(downButton);
  int pushButtonState = digitalRead(pushButton);
  int upButtonState = digitalRead(upButton);
  // print out the state of the button:
  if (rightButtonState){
     Serial.println("Right");
     dispSerial.write(254); //escape character
     dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display
     dispSerial.print("R");
  }
  if (leftButtonState){
     Serial.println("Left");
     dispSerial.write(254); //escape character
     dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display
     dispSerial.print("L");
  }
  if (downButtonState){
     Serial.println("Down");
     dispSerial.write(254); //escape character
     dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display
     dispSerial.print("D");
  }
  if (pushButtonState){
     Serial.println("Button");
     dispSerial.write(254); //escape character
     dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display
     dispSerial.print("B");
  }
  if (upButtonState){
     Serial.println("Up");
     dispSerial.write(254); //escape character
     dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display
     dispSerial.print("U");
  }
  if (!upButtonState | !pushButtonState | !leftButtonState | !rightButtonState | !downButtonState){
     dispSerial.write(254); //escape character
     dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display
     dispSerial.print(" ");
  }
  delay(10);        // delay in between reads for stability
}



