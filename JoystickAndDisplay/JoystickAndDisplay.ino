//#include <AltSoftSerial.h>

#include <SoftwareSerial.h>
#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

//AltSoftSerial dispSerial;

//Set up the display
SoftwareSerial dispSerial(8,9);

// enable the CAN interface with the MCP2515 chip
MCP_CAN CAN0(10); 

char line1_1chars[8] = {' ',' ',' ',' ',' ',' ',' ',' '};
char line1_2chars[8] = {' ',' ',' ',' ',' ',' ',' ',' '};
char line2_1chars[8] = {' ',' ',' ',' ',' ',' ',' ',' '};
char line2_3chars[8] = {' ',' ',' ',' ',' ',' ',' ',' '};
 
byte joyMessage[8];
byte mode;
byte numberOfModes = 2;

//CAN interface messages (Borrowed from the example).
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

//Define the button input pins
const int rightButton = 14;
const int downButton = 15;
const int pushButton = 17;
const int leftButton = 5;
const int upButton = 3;

boolean rightButtonReading = LOW;
boolean leftButtonReading = LOW;
boolean downButtonReading = LOW;
boolean pushButtonReading = LOW;
boolean upButtonReading = LOW;

boolean rightButtonState = LOW;
boolean leftButtonState = LOW;
boolean downButtonState = LOW;
boolean pushButtonState = LOW;
boolean upButtonState = LOW;

boolean doubleClick = false;

boolean lastRightButtonState = LOW;
boolean lastLeftButtonState = LOW;
boolean lastDownButtonState = LOW;
boolean lastPushButtonState = LOW;
boolean lastUpButtonState = LOW;

//Set up various timers
unsigned long currentMillis = 0;
unsigned long lastRightButtonDebounceTime = 0;
unsigned long lastLeftButtonDebounceTime = 0;
unsigned long lastDownButtonDebounceTime = 0;
unsigned long lastPushButtonDebounceTime = 0;
unsigned long lastUpButtonDebounceTime = 0;
unsigned long doubleClickTimer = 0;
unsigned long tripleClickTimer = 0;
unsigned long lastSentTime = 0;
unsigned long lastJoyTime = 0;
unsigned long loopCount = 0;
unsigned long delayItime = 0;
unsigned long lastOffDisplayTime = 0;

// setup user interface times in milliseconds
const long debounceDelay = 20;
const long doubleClickThreshold = 350;
const long tripleClickThreshold = 350;

void setup() {

  pinMode(2, INPUT_PULLUP); //Monitor for CAN messages
  pinMode(3, INPUT_PULLUP); //Monitor for CAN messages

  attachInterrupt(digitalPinToInterrupt(2), readCANbus, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), readCANbus, FALLING);
  
  Serial.begin(115200);
  Serial.println("It's time to go fishing with the Dailys!!");

  //start CAN communications
  Serial.println("Setting up CAN0..."); //J1939
  if(CAN0.begin(CAN_500KBPS) == CAN_OK) Serial.println("CAN0 init ok!!");
  else Serial.println("CAN0 init fail!!");

  CAN0.init_Filt(0,1,0x211);
  CAN0.init_Filt(0,1,0x212);
  CAN0.init_Filt(0,1,0x221);
  CAN0.init_Filt(0,1,0x222);

  // make the pushbutton's pin an input:
  pinMode(rightButton, INPUT);
  pinMode(leftButton, INPUT);
  pinMode(downButton, INPUT);
  pinMode(pushButton, INPUT);
  pinMode(upButton, INPUT);

  dispSerial.begin(9600);
  delay(10);

  // dispSerial.write(0x7C);
  // dispSerial.write(157); //Full Brightness

  dispSerial.write(254);
  dispSerial.write(1); //clear screen


  dispSerial.write(254); 
  dispSerial.write(0x0C); //Turn cursor off

  dispSerial.write(254); // move cursor to beginning of first line (254, 128)
  dispSerial.write(128);
  delay(10);
  dispSerial.print("Let's go fishing");
  dispSerial.print("Fun for everyone");

}

// the loop routine runs over and over again forever:
void loop() {
  //readCANbus();
  

  //Button Debouncing*********************************************************************************    
  // read the input pin:
  rightButtonReading = digitalRead(rightButton);
  leftButtonReading  = digitalRead(leftButton);
  downButtonReading  = digitalRead(downButton);
  pushButtonReading  = digitalRead(pushButton);
  upButtonReading    = digitalRead(upButton);
  // print out the state of the button:
  currentMillis = millis();
  if (rightButtonReading != lastRightButtonState) lastRightButtonDebounceTime = currentMillis;
  if (leftButtonReading  != lastLeftButtonState)  lastLeftButtonDebounceTime  = currentMillis;
  if (downButtonReading  != lastDownButtonState)  lastDownButtonDebounceTime  = currentMillis;
  if (upButtonReading    != lastUpButtonState)    lastUpButtonDebounceTime    = currentMillis;
  if (pushButtonReading  != lastPushButtonState)  lastPushButtonDebounceTime  = currentMillis;  

  if (currentMillis - lastRightButtonDebounceTime > debounceDelay){
    if (rightButtonReading != rightButtonState){
      rightButtonState = rightButtonReading;
    }
  }
  if (currentMillis - lastLeftButtonDebounceTime > debounceDelay){
    if (leftButtonReading != leftButtonState){
      leftButtonState = leftButtonReading;
    }
  }
  if (currentMillis - lastDownButtonDebounceTime > debounceDelay){
    if (downButtonReading != downButtonState){
      downButtonState = downButtonReading;
    }
  }
  if (currentMillis - lastUpButtonDebounceTime > debounceDelay){
    if (upButtonReading != upButtonState){
      upButtonState = upButtonReading;
    }
  }
  if (currentMillis - lastPushButtonDebounceTime > debounceDelay){
    if (pushButtonReading !=  pushButtonState){
      pushButtonState = pushButtonReading;
      if (pushButtonState == HIGH){
        //Serial.println("Push");
//        dispSerial.write(254); //escape character
//        dispSerial.write(206); //Move Cursor to the bottom last point on a 16x2 display
//        dispSerial.print("B");
        if (currentMillis - doubleClickTimer < doubleClickThreshold){
          doubleClick = true;
          mode += 1;
          if (mode >= numberOfModes) mode = 0;
        }
        else {
          doubleClick = false;
          doubleClickTimer = currentMillis;
        }
      }
      else {

//        dispSerial.write(254); //escape character
//        dispSerial.write(206); //Move Cursor to the bottom last point on a 16x2 display
//        dispSerial.print(" ");
      }
    }
  }
  lastRightButtonState = rightButtonReading;
  lastLeftButtonState  = leftButtonReading;
  lastDownButtonState  = downButtonReading;
  lastUpButtonState    = upButtonReading;
  lastPushButtonState  = pushButtonReading;
 
  
  sendJoyStick();
}


/***********************************************************************************************/
/***********************************************************************************************/
void sendJoyStick(){
  currentMillis=millis();
  if (currentMillis - lastJoyTime >=50){
    lastJoyTime = currentMillis;
    joyMessage[0] = byte(mode);
    bitWrite(joyMessage[1],0,upButtonState);
    bitWrite(joyMessage[1],1,downButtonState);
    bitWrite(joyMessage[1],2,leftButtonState);
    bitWrite(joyMessage[1],3,rightButtonState);
    bitWrite(joyMessage[1],4,pushButtonState);
    bitWrite(joyMessage[1],5,doubleClick);
    bitWrite(joyMessage[1],6,0);
    bitWrite(joyMessage[1],7,0);
    
    CAN0.sendMsgBuf(0x007, 0, 2, joyMessage );
  }
}
/***********************************************************************************************/
/***********************************************************************************************/


/***********************************************************************************************/
/***********************************************************************************************/

void readCANbus(){
  //while(CAN0.checkReceive()){
    CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
    rxId = CAN0.getCanId();   
    // Get message ID
//    Serial.print(rxId, HEX);
//    for (int i = 0;i<len;i++){
//      
//      char hexChars[5];
//      sprintf(hexChars,", %02X",rxBuf[i]);
//      Serial.print(hexChars);
//    }
//    Serial.println();
     
    if (rxId == 0x210){ //Mode Message
      numberOfModes = rxBuf[0];
      if (rxBuf[1] <= numberOfModes) mode = rxBuf[1];
    }
    else if (rxId == 0x211){ //Display Characters on first quarter of screen
      dispSerial.write(254); //escape character
      dispSerial.write(128); //Move Cursor
      char str11[9] = {rxBuf[0],rxBuf[1],rxBuf[2],rxBuf[3],rxBuf[4],rxBuf[5],rxBuf[6],rxBuf[7],'\0'};
      dispSerial.print(str11);
      //Serial.println(str11);
    }
    else if (rxId == 0x212){
      dispSerial.write(254); //escape character
      dispSerial.write(136); //Move 
      char str12[9] = {rxBuf[0],rxBuf[1],rxBuf[2],rxBuf[3],rxBuf[4],rxBuf[5],rxBuf[6],rxBuf[7],'\0'};
      dispSerial.print(str12);
      //Serial.println(str12);
        
    }
    else if (rxId == 0x221){
      dispSerial.write(254); //escape character
      dispSerial.write(192); //Move 
      char str21[9] = {rxBuf[0],rxBuf[1],rxBuf[2],rxBuf[3],rxBuf[4],rxBuf[5],rxBuf[6],rxBuf[7],'\0'};
      dispSerial.print(str21);
      //Serial.println(str21);
    }
    else if (rxId == 0x222){
      dispSerial.write(254); //escape character
      dispSerial.write(200); //Move 
      char str22[9] = {rxBuf[0],rxBuf[1],rxBuf[2],rxBuf[3],rxBuf[4],rxBuf[5],rxBuf[6],rxBuf[7],'\0'};
      dispSerial.print(str22);
      //Serial.println(str22);
    }
    else {
     rxId = 0;
    }
  //}
}

