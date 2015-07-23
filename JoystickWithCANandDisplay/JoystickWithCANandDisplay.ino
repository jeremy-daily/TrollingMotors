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

int turnRate = 1000; // This is the number of milliseconds per degree in a turn.

int K = 64; //Proportional gain Constant. This is multiplied by the number of degrees difference.
int I = 64; // Integral gain Constant. Since this is integer math, bit shifting is easier to implement.
int D = 64; // Derivative gain Constant. Since this is integer math, bit shifting is needed to get fractions.

//Set up bit shifts of 6 bits to make division by 64 
int bitShiftK = 6;
int bitShiftI = 14; // Example bit shift of 8 = division by 256. Depends on the Memory Size
int bitShiftD = 6;

//feed forward value to command a turn
int leftTurn = -15;
int rightTurn = 15; 

int turnAdjust = 0; // feed forward value to command a turn

int usedI=0;
int usedK=0;
int usedD=0;

const int memorySize=256;
int differenceList[256];

int deltaT = 1000; //milliseconds to calculate heading rate changes

int zeroAdjustR = 0; //This is a constant to make sure the motors are sent back to zero.
int zeroAdjustL = 0; //This is a constant to make sure the motors are sent back to zero.

int turnSetting = 15; //

const int headingFixedOffset = 110; //This accounts for the orientation of the compass in the boat in tenths of a degree.
int degreeCounter =0;

//values used to send CAN message to the motor Controller
int rightMotor = 0;
int leftMotor = 0;
byte motorMessage[2];

byte joyMessage[2];
    
//CAN interface messages (Borrowed from the example).
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];


//Set up modes of operation
int mode = 0; 
int numberOfModes = 6; //This limits the number of displayed modes.
char modeNames[6][6]={"Off ","Man. ","Turn ","Fix ","Fig8 ", "Full"};

//Set up the integrator to compensate for drift and error in the PID loop

int diffIndex=0;
long sum = 0;

int finalHeading = 0; // the goal for the turns

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

boolean lastRightButtonState = LOW;
boolean lastLeftButtonState = LOW;
boolean lastDownButtonState = LOW;
boolean lastPushButtonState = LOW;
boolean lastUpButtonState = LOW;

//Set up various timers
long currentMillis = 0;
long lastDegreeTime = 0;
long lastHeadingTime = 0;
long lastCalculateTime = 0;
long turnChangeTimer = 0;
long lastRightButtonDebounceTime = 0;
long lastLeftButtonDebounceTime = 0;
long lastDownButtonDebounceTime = 0;
long lastPushButtonDebounceTime = 0;
long lastUpButtonDebounceTime = 0;
long doubleClickTimer = 0;
long tripleClickTimer = 0;
long speedChangeTimer = 0;
long goalChangeTimer = 0;
long lastModeDisplayTime = 0;
long lastDesireDisplayTime = 0;
long lastReadingDisplayTime = 0;
long lastDisplayTime = 0;
long lastSentTime = 0;
long lastJoyTime = 0;
long loopCount = 0;
long delayItime = 0;

// setup user interface times in milliseconds
const long debounceDelay = 20;
const long doubleClickThreshold = 350;
const long tripleClickThreshold = 350;
const byte speedChangeDelay = 50;
const byte goalChangeDelay = 60;

boolean modeEnable = false;
boolean firstHeading = true;

// Set up operational Variables
int voltage = 0;
int portVoltage = 0;
int starVoltage = 0;
int difference = 0;
int lastHeading = 0;
int headingChange = 0;
int speedSetting = 0;
int goalSetting = 0;
int currentHeading = 0;
int headingReading = 0;
int startHeading = 0;
int heading1 = 0;
int gpsSpeed = 0;
int gpsAngle = 0;
byte gpsSats = 0;
byte gpsFix = 0;

long currentLongitude = 0;
long desiredLongitude = 0;
long currentLatitude = 0;
long desiredLatitude = 0;

/***********************************************************************************************/
/***********************************************************************************************/
// the setup routine runs once
void setup() {
 
  pinMode(2, INPUT); //Monitor for CAN messages
 
  Serial.begin(115200);
  Serial.println("It's time to go fishing with the Dailys!!");
  
  //start CAN communications
  Serial.println("Setting up CAN0..."); //J1939
  if(CAN0.begin(CAN_500KBPS) == CAN_OK) Serial.println("CAN0 init ok!!");
  else Serial.println("CAN0 init fail!!");
  
//  Serial.println("Possible Modes are:");
//  for (int g = 0; g < numberOfModes; g++){
//    Serial.print(g,DEC);
//    Serial.print(": ");
//    Serial.println(modeNames[g]);
//  }
  Serial.print("K = \t");
  Serial.println(K);
  Serial.print("I = \t");
  Serial.println(I);
  Serial.print("D = \t");
  Serial.println(D);
  Serial.print("BitShift K = \t");
  Serial.println(bitShiftK);
  Serial.print("BitShift I = \t");
  Serial.println(bitShiftI);
  Serial.print("BitShift D = \t");
  Serial.println(bitShiftD);
  usedK=K;
  usedI=I;
  usedD=D;
      
  Serial.println("#:Mode\tMillis\tSpeed\tLat\tLong\tHead\tFinal\tGoal\tDiff\tSum\tLeft\tRight");
    
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
  delay(2200);
  
  
  dispSerial.write(254); //escape character
  dispSerial.write(1); //clear Screen
  dispSerial.write(254); //escape character
  dispSerial.write(128); //Move Cursor to beginning of the display
  dispSerial.print(modeNames[mode]);
  
  //initialize or reset all parameters
  modeEnable=false;
  goalSetting = currentHeading;
  speedSetting = 0;
}
/***********************************************************************************************/
/***********************************************************************************************/

// the loop routine runs over and over again forever:
void loop() {
  currentMillis = millis();
  readCANbus();

  /***********************************************************************************************/
  if (mode == 0){ //Off 
    
    if (currentMillis - lastReadingDisplayTime > 286){
        lastReadingDisplayTime = currentMillis;
        char displayBuffer3[11];
        int volts = voltage/10;
        int millivolts = voltage - volts*10;
        sprintf(displayBuffer3,"%2i.%1i volts", volts,millivolts);
        dispSerial.write(254); //escape character
        dispSerial.write(134); //First Line
        dispSerial.print(displayBuffer3);
      }
      displayReadings();
  }
  /***********************************************************************************************/
  else if (mode == 1){ //Manual
    if (pushButtonState) { 
      if (upButtonState)
      {
        goalSetting = currentHeading;
        //speedSetting = 0;
        memset(differenceList,0,sizeof(differenceList)); // clears the integrator
      }
      else if (downButtonState)
      {
        speedSetting = 0;
      }
      if (rightButtonState) {
            incrementGoalby10();
            //usedI = 0;
            //memset(differenceList,0,sizeof(differenceList)); // clears the integrator
            //delayItime = currentMillis;
      }
      else if (leftButtonState)  {
            decrementGoalby10();
            //usedI = 0;
            //memset(differenceList,0,sizeof(differenceList)); // clears the integrator
            //delayItime = currentMillis;
      }
    }
    else
    {
      if (upButtonState) incrementSpeed();
      if (downButtonState) decrementSpeed();
      if (rightButtonState) {
            incrementGoal();
            //usedI = 0;
            //memset(differenceList,0,sizeof(differenceList)); // clears the integrator
            //delayItime = currentMillis;
      }
      else if (leftButtonState)  {
            decrementGoal();
            //usedI = 0;
            //memset(differenceList,0,sizeof(differenceList)); // clears the integrator
            //delayItime = currentMillis;
      }
    }
    

    
    
    computeValues();
    displayDesires();
    displayReadings();
    sendCommands();
    
    //if (currentMillis - delayItime > 5000) usedI = I;
  }
  
  /***********************************************************************************************/
  else if (mode == 2){ //Turn
    
    if (currentMillis - turnChangeTimer > goalChangeDelay){
      turnChangeTimer=currentMillis;
      if (pushButtonState){
        if (upButtonState) {
          finalHeading = currentHeading;
          goalSetting = currentHeading;
          turnAdjust = 0;
        }
        if (downButtonState) {
          goalSetting = currentHeading;
          finalHeading = currentHeading + 1800;
          turnAdjust = rightTurn;
        }
        if (leftButtonState) turnRate -=5;
        if (rightButtonState) turnRate +=5;  
      }
      else{
        if (upButtonState) incrementSpeed();
        if (downButtonState) decrementSpeed();
        if (rightButtonState){
          finalHeading += 900;
          turnChangeTimer += 1000;
          turnAdjust = rightTurn;
        }
        if (leftButtonState){
          finalHeading -= 900;
          turnChangeTimer += 1000;
          turnAdjust = leftTurn;
        }
      }
    }
    
    if (upButtonState) incrementSpeed();
    if (downButtonState) decrementSpeed();
    if (pushButtonState && upButtonState) {
      modeEnable = true;
      speedSetting = 70;
    }
    
    if (modeEnable){
    
      if (currentMillis - lastReadingDisplayTime > 186){
          lastReadingDisplayTime = currentMillis;
          char displayBuffer1[15];
          if (pushButtonState){
            sprintf(displayBuffer1,"END:%3i Rt:%4i", int(finalHeading/10.0), turnRate);
          }
          else
          {
            sprintf(displayBuffer1,"END:%3i H:%3i ", int(finalHeading/10.0),int(currentHeading/10.0));
          }
          dispSerial.write(254); //escape character
          dispSerial.write(192); //Beginning of the second line
          dispSerial.print(displayBuffer1);
        }
      implementTurn();
      computeValues();
      sendCommands();
    }
    
    else{
      dispSerial.write(254); //escape character
      dispSerial.write(192); //Beginning of the second line
      dispSerial.print("Press Button+Up"); 
    }
   displayDesires(); 
  }
  /***********************************************************************************************/

  else if (mode == 3){ //fix
   
    
//   float deltaX = desiredLatitude-currentLatitude;
//   float deltaY = desiredLongitude-currentLongitude;
//   
//   float radialError = sqrt(deltaX*deltaX + deltaY*deltaY);
//   
//   goalSetting = int(RAD_TO_DEG * (atan2(deltaY, deltaX)));
//   computeValues();
//   displayReadings();
//   sendCommands();
//    
//   if (currentMillis - lastReadingDisplayTime > 198){
//        lastReadingDisplayTime = currentMillis;
//        dispSerial.write(254); //escape character
//        dispSerial.write(133); 
//        char displayBuffer[14];
//        sprintf(displayBuffer,"H:%3i G:%3i", currentHeading,goalSetting);
//        dispSerial.print(displayBuffer);
//        dispSerial.write(254); //escape character
//        dispSerial.write(192); //Beginning of the second line
//        dispSerial.print("Dist:");
//        dispSerial.print(radialError);
//      }
  }
  /***********************************************************************************************/

  else if (mode == 4){ //Figure 8
    if (upButtonState) incrementSpeed();
    if (downButtonState) decrementSpeed();
    if (pushButtonState && upButtonState) {
      modeEnable = true;
      speedSetting = 70;
    }
    
    if (modeEnable){
      
      if      (degreeCounter < 90)  finalHeading = startHeading + 900;
      else if (degreeCounter < 180) finalHeading = startHeading + 1800;
      else if (degreeCounter < 270) finalHeading = startHeading + 2700;
      else if (degreeCounter < 360) finalHeading = startHeading + 1800;
      else if (degreeCounter < 450) finalHeading = startHeading + 900;
      else if (degreeCounter < 540) finalHeading = startHeading ;
      else degreeCounter = 0;
      
      if (degreeCounter < 270){
          turnAdjust = rightTurn;
      }
      else { 
        turnAdjust = leftTurn;
      }
      
      if (currentMillis - lastReadingDisplayTime > 186){
        lastReadingDisplayTime = currentMillis;
        char displayBuffer1[15];
        sprintf(displayBuffer1,"END:%3i H:%3i ", int(finalHeading/10.0),int(currentHeading/10.0));
        dispSerial.write(254); //escape character
        dispSerial.write(192); //Beginning of the second line
        dispSerial.print(displayBuffer1);
      }
      
      implementTurn();
      computeValues();
      displayDesires();
      sendCommands();
    }
    else{
      dispSerial.write(254); //escape character
      dispSerial.write(192); //Beginning of the second line
      dispSerial.print("Press Button+Up"); 
    }
    
  }
  
/***********************************************************************************************/
  else if (mode == 5){ //Full Throttle
    usedI=0;
    usedD=0;
    if (upButtonState) {
      speedSetting = 100;
      usedK=0;
    }
    else if (downButtonState) {
      speedSetting = -99;
      usedK=0;
    }
    else if (rightButtonState) {
      goalSetting = currentHeading + 900;
      usedK = 100;
    }
    else if (leftButtonState){
      goalSetting = currentHeading - 900;
      usedK = 100;
    }
    else {
      goalSetting = currentHeading;
      speedSetting=0;
      usedK=0;
    }
        
    
    displayDesires();
    if (currentMillis - lastReadingDisplayTime > 186){
        lastReadingDisplayTime = currentMillis;
        char displayBuffer[15];
        sprintf(displayBuffer,"L:%3i R%3i ", leftMotor,rightMotor);
        dispSerial.write(254); //escape character
        dispSerial.write(192); //Beginning of the second line
        dispSerial.print(displayBuffer);
    }
    computeValues();
    sendCommands();
  
  }
  
 //Button Debouncing*********************************************************************************    
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
         //Serial.println("Right");
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
         //Serial.println("Left");
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
         //Serial.println("Down");
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
         //Serial.println("Up");
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
         //Serial.println("Push");
         dispSerial.write(254); //escape character
         dispSerial.write(206); //Move Cursor to the bottom last point on a 16x2 display
         dispSerial.print("B");
         if (currentMillis - doubleClickTimer < doubleClickThreshold){
           doubleClickRoutine();
         }
         else {
             doubleClickTimer = currentMillis;
         }
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
  sendJoyStick();
}
/***********************************************************************************************/
/***********************************************************************************************/
void implementTurn(){
//Increment 1 degree in the correct direction according to the compass 
    //Considerations for crossing 0/360 are implemented.
   
    if (finalHeading > 3600) finalHeading -= 3600;
    if (finalHeading < 0) finalHeading += 3600;
    if (goalSetting > 3600) goalSetting -= 3600;
    if (goalSetting < 0) goalSetting += 3600;
    
    if (currentMillis - lastDegreeTime > turnRate){
        lastDegreeTime = currentMillis;
        if (abs(finalHeading - goalSetting) < 1800){
          if (finalHeading > goalSetting) goalSetting +=10;
          else if (finalHeading < goalSetting) goalSetting -=10;
          else {
           finalHeading = goalSetting;
           turnAdjust = 0;
          }
        }
        else {
          if      (finalHeading < goalSetting && finalHeading >  1800) goalSetting -=10;
          else if (finalHeading < goalSetting && finalHeading <= 1800) goalSetting +=10;
          else if (finalHeading > goalSetting && finalHeading >  1800) goalSetting -=10;
          else if (finalHeading > goalSetting && finalHeading <= 1800) goalSetting +=10;
          else finalHeading = goalSetting;
        }
        degreeCounter+=1;
        if (degreeCounter > 540) degreeCounter = 0;
    }
}
//Double Clikcing changes mode and resets settings
void doubleClickRoutine(){ 
  mode += 1;
  if (mode >= numberOfModes) mode = 0;
  dispSerial.write(254); //escape character
  dispSerial.write(1); //Clear Screen
  dispSerial.write(254); //escape character
  dispSerial.write(128); //Move Cursor to beginning of the display
  dispSerial.print(modeNames[mode]);
  modeEnable=false;
  goalSetting = currentHeading;
  finalHeading = currentHeading;
  startHeading = currentHeading;
  speedSetting = 0;
  zeroAdjustR = 0;
  zeroAdjustL = 0;
  turnSetting = 0; 

  usedI = I;
  usedK = K;
  usedD = D;
  memset(differenceList,0,sizeof(differenceList)); // clears the integrator
}
/***********************************************************************************************/
/***********************************************************************************************/
void sendCommands(){
  currentMillis=millis();
  if (currentMillis - lastSentTime >=200){
    lastSentTime = currentMillis;
  
    
    Serial.print(mode);
    Serial.print(":");
    Serial.print(modeNames[mode]);
    Serial.print("\t");
    Serial.print(currentMillis);
    Serial.print("\t");
    Serial.print(speedSetting);
    Serial.print("\t");
    Serial.print(currentLatitude);
    Serial.print("\t");
    Serial.print(currentLongitude);
    Serial.print("\t");
    Serial.print(currentHeading);
    Serial.print("\t");
    Serial.print(finalHeading);
    Serial.print("\t");
    Serial.print(goalSetting);
    Serial.print("\t");
    Serial.print(difference);
    Serial.print("\t");
    Serial.print(sum);
    Serial.print("\t");
    Serial.print(leftMotor);
    Serial.print("\t");
    Serial.println(rightMotor);
    
    motorMessage[0]=byte(rightMotor);
    motorMessage[1]=byte(leftMotor);
    CAN0.sendMsgBuf(0x31A, 0, 2, motorMessage );
  }
}
/***********************************************************************************************/
/***********************************************************************************************/
void sendJoyStick(){
  currentMillis=millis();
  if (currentMillis - lastJoyTime >=50){
    lastJoyTime = currentMillis;

    joyMessage[0]=byte(mode);
    bitWrite(joyMessage[1],0,upButtonState);
    bitWrite(joyMessage[1],1,downButtonState);
    bitWrite(joyMessage[1],2,leftButtonState);
    bitWrite(joyMessage[1],3,rightButtonState);
    bitWrite(joyMessage[1],4,pushButtonState);
    bitWrite(joyMessage[1],7,modeEnable);
    
    CAN0.sendMsgBuf(0x777, 0, 2, joyMessage );
  }
}
/***********************************************************************************************/
/***********************************************************************************************/
void incrementSpeed(){
  if (currentMillis - speedChangeTimer > speedChangeDelay){
    speedChangeTimer=currentMillis;
    speedSetting +=1;
    if (speedSetting > 100) speedSetting = 100;
  }
}
/***********************************************************************************************/
/***********************************************************************************************/
void decrementSpeed(){
  if (currentMillis - speedChangeTimer > speedChangeDelay){
    speedChangeTimer=currentMillis;
    speedSetting -=1;
    if (speedSetting < -99) speedSetting = -99;
  }
} 
/***********************************************************************************************/
/***********************************************************************************************/
void incrementGoal(){
  if (currentMillis - goalChangeTimer > goalChangeDelay){
    goalChangeTimer=currentMillis;
    goalSetting +=10;
    if (goalSetting > 3600) goalSetting = 1;
  }
}
/***********************************************************************************************/
/***********************************************************************************************/
void decrementGoal(){
  if (currentMillis - goalChangeTimer > goalChangeDelay){
    goalChangeTimer=currentMillis;
    goalSetting -=10;
    if (goalSetting < 0) goalSetting = 3599;
  }
} 
/***********************************************************************************************/
/***********************************************************************************************/
void incrementGoalby10(){
  if (currentMillis - goalChangeTimer > goalChangeDelay){
    goalChangeTimer=currentMillis+1000;
    goalSetting +=100;
    if (goalSetting > 3600) goalSetting = 1;
  }
}
/***********************************************************************************************/
/***********************************************************************************************/
void decrementGoalby10(){
  if (currentMillis - goalChangeTimer > goalChangeDelay){
    goalChangeTimer=currentMillis+1000;
    goalSetting -=100;
    if (goalSetting < 0) goalSetting = 3599;
  }
} 
/***********************************************************************************************/
/***********************************************************************************************/
void readCANbus(){
  CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
  rxId = CAN0.getCanId();                    // Get message ID
//  Serial.print("ID: ");
//  Serial.print(rxId, HEX);
//  Serial.print("  Data: ");
//  for(int i = 0; i<len; i++)                // Print each byte of the data
//  {
//    if(rxBuf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
//    {
//      Serial.print("0");
//    }
//    Serial.print(rxBuf[i], HEX);
//    Serial.print(" ");
//  }
//  Serial.println();
   if (rxId == 0x43c){
     heading1 = (rxBuf[0]*256 + rxBuf[1]);
   }
   else if (rxId == 0x43d){
     int vSenseReading = rxBuf[0]*256 + rxBuf[1];
     int portVsenseReading = rxBuf[2]*256 + rxBuf[3];
     int starVsenseReading = rxBuf[4]*256 + rxBuf[5];
     //heading1 = (rxBuf[6]*256 + rxBuf[7])/10.;
     voltage     = map(vSenseReading,0,630,0,120);
     portVoltage = map(portVsenseReading,0,630,0,120);
     starVoltage = map(starVsenseReading,0,630,0,120);
   }
   else if (rxId == 0x43e){
     headingReading = (rxBuf[0]*256 + rxBuf[1]);
     gpsSpeed = (rxBuf[2]*256 + rxBuf[3])*1.15078;
     gpsAngle = (rxBuf[4]*256 + rxBuf[5]);
     gpsSats  = rxBuf[6];
     gpsFix   = rxBuf[7];
   }
   else if (rxId == 0x43f){
     long temp0 = rxBuf[0];
     long temp1 = rxBuf[1];
     long temp2 = rxBuf[2];
     long temp3 = rxBuf[3];
     currentLatitude = temp0 << 24 + temp1 << 16 + temp2 << 8 + temp3;
     temp0 = rxBuf[4];
     temp1 = rxBuf[5];
     temp2 = rxBuf[6];
     temp3 = rxBuf[7];
     currentLongitude = temp0 << 24 + temp1 << 16 + temp2 << 8 + temp3;
     
   }
  
  currentHeading = int(heading1 + headingFixedOffset);
  
  if (currentHeading > 3600) currentHeading -= 3600;
  if (currentHeading < 0) currentHeading += 3600;
  
  if (firstHeading){
    goalSetting = currentHeading;
    firstHeading = false;
  }
}
/***********************************************************************************************/
/***********************************************************************************************/
void displayDesires(){
  if (currentMillis - lastDesireDisplayTime > 98){
    lastDesireDisplayTime = currentMillis;
    dispSerial.write(254); //escape character
    dispSerial.write(133); 
    char displayBuffer[14];
    sprintf(displayBuffer,"S:%3i G:%3i", speedSetting,int(goalSetting/10));
    dispSerial.print(displayBuffer);
  }
}
/***********************************************************************************************/
/***********************************************************************************************/
void displayReadings(){
  if (currentMillis - lastReadingDisplayTime > 183){
    lastReadingDisplayTime = currentMillis;
    dispSerial.write(254); //escape character
    dispSerial.write(192); //Beginning of the second line
    char displayBuffer[18];
    sprintf(displayBuffer,"N%2i S:%2i H:%3i", gpsSats,gpsSpeed,int(currentHeading/10.0));
    dispSerial.print(displayBuffer);
//    dispSerial.print("N:");
//    dispSerial.print(gpsSats);
//    dispSerial.print(" S:");
//    dispSerial.print(gpsSpeed);
//    dispSerial.print(" H:");
//    dispSerial.print(currentHeading);
//    dispSerial.print("    ");
  }
}
/***********************************************************************************************/
/***********************************************************************************************/
void  computeValues(){
  currentMillis = millis();
  if (currentMillis - lastCalculateTime > 50){
        lastCalculateTime = currentMillis;
        
        if (currentMillis - lastHeadingTime > deltaT){
          lastHeadingTime = currentMillis;
          headingChange = currentHeading - lastHeading ;
          if (headingChange <= -1800)  headingChange += 3600;
          else if (headingChange > 1800)  headingChange -= 3600;
          lastHeading = currentHeading;
        }
       
        difference = goalSetting - currentHeading;
        if (difference <= -1800)  difference += 3600;
        if (difference >= 1800)  difference -= 3600;
        
        differenceList[diffIndex] = difference;
        diffIndex+=1;
        if (diffIndex >= memorySize) diffIndex = 0;
        
        sum = 0;
        for (int j = 0; j < memorySize; j++){
          sum += differenceList[j];
        }

        int tempRightMotor = speedSetting - turnSetting - ((usedK*difference) >> bitShiftK) - zeroAdjustR - ((usedI*sum) >> bitShiftI) - ((usedD*headingChange)>> bitShiftD) + 100;
        int tempLeftMotor  = speedSetting + turnSetting + ((usedK*difference) >> bitShiftK) + zeroAdjustL + ((usedI*sum) >> bitShiftI) + ((usedD*headingChange)>> bitShiftD) + 100;
        rightMotor = constrain(tempRightMotor,0,200);
        leftMotor  = constrain(tempLeftMotor,0,200);
     }       
}
/***********************************************************************************************/
/***********************************************************************************************/
//Todo
//void calculateSpeeds(){
//  currentMillis = millis();
//  if (currentMillis - lastCalculateSpeedTime > 250){
//        lastCalculateSpeedTime = currentMillis;
//        difference = goalSetting - currentHeading;
//       
//        if (currentMillis - lastHeadingTime > deltaT){
//          lastHeadingTime = currentMillis;
//          headingChange = currentHeading - lastHeading ;
//          lastHeading = currentHeading;
//        }
//        
//        if (rightMotor < 200 && leftMotor < 200){ // Reduces Integral windup when the actuator(motor) is saturated.
//          speedDifferenceList[diffIndex] = difference;
//          speedDiffIndex+=1;
//          if (speedDiffIndex >= speedMemorySize) diffIndex = 0;
//        
//          speedSum = 0;
//          for (int j = 0; j < speedMemorySize; j++){
//            speedSum += speedDifferenceList[j];
//          } 
//          
//        
//          
//        }
//
//     
//  }
//}
