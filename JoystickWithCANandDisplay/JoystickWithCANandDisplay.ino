#include <AltSoftSerial.h>

AltSoftSerial dispSerial;
/*
  DigitalReadSerial
 Reads a digital input on pin 2, prints the result to the serial monitor 
 
 This example code is in the public domain.
 */


const int rightButton = 14;
const int downButton = 15;
const int pushButton = 16;
const int leftButton = 2;
const int upButton = 3;

boolean rightButtonReading = LOW;
boolean leftButtonReading = LOW;
boolean downButtonReading = LOW;
boolean pushButtonReading = LOW;
boolean upButtonReading = LOW;

boolean rightButtonState = LOW;
boolean leftButtonState = LOW;
boolean downButtonState = LOW;
boolean pushButtonState = 0;
boolean upButtonState = LOW;

boolean lastRightButtonState = LOW;
boolean lastLeftButtonState = LOW;
boolean lastDownButtonState = LOW;
boolean lastPushButtonState = LOW;
boolean lastUpButtonState = LOW;

unsigned long currentMillis = 0;
unsigned long lastRightButtonDebounceTime = 0;
unsigned long lastLeftButtonDebounceTime = 0;
unsigned long lastDownButtonDebounceTime = 0;
unsigned long lastPushButtonDebounceTime = 0;
unsigned long lastUpButtonDebounceTime = 0;

unsigned long lastDisplayTime = 0;

const unsigned long debounceDelay = 50;


byte mode = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at bits per second:
  Serial.begin(115200);
  Serial.println("It's time to go fishing with the Dailys.");
  
  pinMode(A3, OUTPUT); //Enable the switches. Don't delete this.
  digitalWrite(A3,HIGH);
  
  // make the pushbutton's pin an input:
  pinMode(rightButton, INPUT);
  pinMode(leftButton, INPUT);
  pinMode(downButton, INPUT);
  pinMode(pushButton, INPUT);
  pinMode(upButton, INPUT);
  
  dispSerial.begin(9600);
  delay(10);
  dispSerial.write(254); // move cursor to beginning of first line
  dispSerial.write(128);
  delay(10);
  dispSerial.print("It's time to go ");
  dispSerial.print("fishing, Sarah. ");
  
}

// the loop routine runs over and over again forever:
void loop() {
  currentMillis = millis();
//  if (mode == 1){
//  
//  }
//  else if (mode == 2){
//  
//  }
//  else {
//    dispSerial.write(254); // move cursor to beginning of first line
//    dispSerial.write(128);
//    dispSerial.print("Idle Mode       ");
//    dispSerial.print("Press a Button. ");  
//  }
  
  //Do this Always  
  // read the input pin:
  rightButtonReading = digitalRead(rightButton);
  leftButtonReading  = digitalRead(leftButton);
  downButtonReading  = digitalRead(downButton);
  pushButtonReading  = digitalRead(pushButton);
  upButtonReading    = digitalRead(upButton);
  // print out the state of the button:
  
  if (rightButtonReading != lastRightButtonState)    lastRightButtonDebounceTime = currentMillis;
  else if (leftButtonReading != lastLeftButtonState) lastLeftButtonDebounceTime  = currentMillis;
  else if (downButtonReading != lastDownButtonState) lastDownButtonDebounceTime  = currentMillis;
  else if (upButtonReading != lastUpButtonState)     lastUpButtonDebounceTime    = currentMillis;
  if (pushButtonReading != lastPushButtonState)      lastPushButtonDebounceTime  = currentMillis;  
  
  if (currentMillis - lastRightButtonDebounceTime > debounceDelay){
    if (rightButtonReading != rightButtonState){
      rightButtonState = rightButtonReading;
      if (rightButtonState == HIGH){
         Serial.println("Right");
         dispSerial.write(254); //escape character
         dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display
         dispSerial.print("R");
      }
    }
  }
  if (currentMillis - lastLeftButtonDebounceTime > debounceDelay){
    if (leftButtonReading != leftButtonState){
      leftButtonState = leftButtonReading;
      if (leftButtonState == HIGH){
         Serial.println("Left");
         dispSerial.write(254); //escape character
         dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display
         dispSerial.print("L");
      }
    }
  }
 if (currentMillis - lastDownButtonDebounceTime > debounceDelay){
    if (downButtonReading != downButtonState){
      downButtonState = downButtonReading;
      if (downButtonState == HIGH){
         Serial.println("Down");
         dispSerial.write(254); //escape character
         dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display
         dispSerial.print("D");
      }
    }
  }
 if (currentMillis - lastUpButtonDebounceTime > debounceDelay){
    if (upButtonReading != upButtonState){
      upButtonState = upButtonReading;
      if (upButtonState == HIGH){
         Serial.println("Up");
         dispSerial.write(254); //escape character
         dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display
         dispSerial.print("U");
      }
    }
  }
 if (currentMillis - lastPushButtonDebounceTime > debounceDelay){
    if (pushButtonReading != pushButtonState){
      pushButtonState = pushButtonReading;
      if (pushButtonState == HIGH){
         Serial.println("Push");
         dispSerial.write(254); //escape character
         dispSerial.write(206); //Move Cursor to the bottom last point on a 16x2 display
         dispSerial.print("B");
      }
      else {
         dispSerial.write(254); //escape character
         dispSerial.write(206); //Move Cursor to the bottom last point on a 16x2 display
         dispSerial.print(" ");
      }
    }
  }
  lastRightButtonState = rightButtonReading;
  lastLeftButtonState  = leftButtonReading;
  lastDownButtonState  = downButtonReading;
  lastUpButtonState    = upButtonReading;
  lastPushButtonState  = pushButtonReading;
  
  if (!upButtonState & !leftButtonState & !rightButtonState & !downButtonState){
     dispSerial.write(254); //escape character
     dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display
     dispSerial.print(" ");
  }
  
  if (currentMillis - lastDisplayTime > 500){
    lastDisplayTime = currentMillis;
    Serial.print("Switch States: Button = ");
    Serial.print(pushButtonState);
    Serial.print(", Left = ");
    Serial.print(leftButtonState);
    Serial.print(", Right = ");
    Serial.print(rightButtonState);
    Serial.print(", Up = ");
    Serial.print(upButtonState);
    Serial.print(", Down = ");
    Serial.println(downButtonState);
  }
}



