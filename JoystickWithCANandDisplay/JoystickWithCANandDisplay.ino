#include <SoftwareSerial.h>
#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

//Set up the display
SoftwareSerial dispSerial(8,9);

// enable the CAN interface with the MCP2515 chip
MCP_CAN CAN0(10); 

int turnRate = 1000; // This is the number of milliseconds per degree in a turn.

int K = 2; //Proportional gain Constant. This is multiplied by the number of degrees difference.
int I = 1; // Integral gain Constant. Since this is integer math, bit shifting is easier to implement.

const int zeroAdjust = 0; //This is a constant to make sure the motors are sent back to zero.

const int headingFixedOffset = -15; //This accounts for the orientation of the compass in the boat in degrees.

//values used to send CAN message to the motor Controller
int rightMotor = 0;
int leftMotor = 0;
byte motorMessage[2];

//CAN interface messages (Borrowed from the example).
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];


//Set up modes of operation
byte mode = 0; 
int numberOfModes = 3; //This limits the number of displayed modes.
char modeNames[5][6]={"Off ","Man. ","Turn ","Fig8 ","Fix "};

//Set up the integrator to compensate for drift and error in the PI loop
const int memorySize=500;
int differenceList[500];
int diffIndex=0;
long sum = 0;
int drift = 0;

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
boolean pushButtonState = 0;
boolean upButtonState = LOW;

boolean lastRightButtonState = LOW;
boolean lastLeftButtonState = LOW;
boolean lastDownButtonState = LOW;
boolean lastPushButtonState = LOW;
boolean lastUpButtonState = LOW;

//Set up various timers
long currentMillis = 0;
long lastDegreeTime = 0;
long lastCalculateTime = 0;
long turnChangeTimer = 0;
long lastRightButtonDebounceTime = 0;
long lastLeftButtonDebounceTime = 0;
long lastDownButtonDebounceTime = 0;
long lastPushButtonDebounceTime = 0;
long lastUpButtonDebounceTime = 0;
long doubleClickTimer = 0;
long speedChangeTimer = 0;
long goalChangeTimer = 0;
long lastModeDisplayTime = 0;
long lastDesireDisplayTime = 0;
long lastReadingDisplayTime = 0;
long lastDisplayTime = 0;
long lastSentTime = 0;
long loopCount = 0;

// setup user interface times in milliseconds
const long debounceDelay = 20;
const long doubleClickThreshold = 350;
const byte speedChangeDelay = 50;
const byte goalChangeDelay = 60;

boolean modeEnable = false;
boolean firstHeading = true;

// Set up operational Variables
int difference = 0;
int speedSetting = 0;
int goalSetting = 0;
int currentHeading = 0;
int headingReading = 0;
int heading1 = 0;
int gpsSpeed = 0;
int gpsAngle = 0;
byte gpsSats = 0;
byte gpsFix = 0;

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
  
  Serial.println("Possible Modes are:");
  for (int g = 0; g < numberOfModes; g++){
    Serial.print(g,DEC);
    Serial.print(": ");
    Serial.println(modeNames[g]);
  }

  // make the pushbutton's pin an input:
  pinMode(rightButton, INPUT);
  pinMode(leftButton, INPUT);
  pinMode(downButton, INPUT);
  pinMode(pushButton, INPUT);
  pinMode(upButton, INPUT);
  
  dispSerial.begin(9600);
  delay(10);
  dispSerial.write(254); // move cursor to beginning of first line (254, 128)
  dispSerial.write(128);
  delay(10);
  dispSerial.print("Let's go fishing");
  dispSerial.print("Fun for everyone");
  delay(1200);
  
  
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
  if(!digitalRead(2)) readCANbus();

  /***********************************************************************************************/
  if (mode == 0){ //Off 
    displayReadings();
  }
  /***********************************************************************************************/
  else if (mode == 1){ //Manual
    computeValues();
    if (upButtonState) incrementSpeed();
    if (downButtonState) decrementSpeed();
    if (rightButtonState) incrementGoal();
    if (leftButtonState)  decrementGoal();
    if (pushButtonState) { 
      goalSetting = currentHeading;
      speedSetting = 0;
    }
    displayDesires();
    displayReadings();
    sendCommands();
  }
  /***********************************************************************************************/
  else if (mode == 2){ //Turn
    
    if (currentMillis - turnChangeTimer > goalChangeDelay){
      turnChangeTimer=currentMillis;
      if (pushButtonState){
        if (upButtonState) {
          finalHeading = currentHeading;
          goalSetting = currentHeading;
        }
        if (downButtonState) {
          finalHeading = currentHeading;
          goalSetting = currentHeading;
          finalHeading +=180;
        }
        if (leftButtonState) turnRate -=5;
        if (rightButtonState) turnRate +=5;  
      }
      else{
        if (upButtonState) incrementSpeed();
        if (downButtonState) decrementSpeed();
        if (rightButtonState){
          finalHeading +=45;
          turnChangeTimer += 1500;
        }
        if (leftButtonState){
          finalHeading -=45;
          turnChangeTimer += 1500;
        }
      }
    }
    
    if (finalHeading > 360) finalHeading -= 360;
    if (finalHeading < 0) finalHeading += 360;
    if (goalSetting > 360) goalSetting -= 360;
    if (goalSetting < 0) goalSetting += 360;
  
    if (currentMillis - lastReadingDisplayTime > 186){
        lastReadingDisplayTime = currentMillis;
        char displayBuffer1[15];
        if (pushButtonState){
          sprintf(displayBuffer1,"Rate:%4i      ", turnRate);
        }
        else
        {
          sprintf(displayBuffer1,"H:%3i END:%3i", currentHeading,finalHeading);
        }
        dispSerial.write(254); //escape character
        dispSerial.write(192); //Beginning of the second line
        dispSerial.print(displayBuffer1);
      }
    
    //Increment 1 degree in the correct direction according to the compass 
    //Considerations for crossing 0/360 are implemented.
    if (currentMillis - lastDegreeTime > turnRate){
        lastDegreeTime = currentMillis;
        if (abs(finalHeading - goalSetting) < 180){
          if (finalHeading > goalSetting) goalSetting +=1;
          else if (finalHeading < goalSetting) goalSetting -=1;
          else finalHeading = goalSetting;
        }
        else {
          if      (finalHeading < goalSetting && finalHeading >  180) goalSetting -=1;
          else if (finalHeading < goalSetting && finalHeading <= 180) goalSetting +=1;
          else if (finalHeading > goalSetting && finalHeading >  180) goalSetting -=1;
          else if (finalHeading > goalSetting && finalHeading <= 180) goalSetting +=1;
          else finalHeading = goalSetting;
        }
    }
    computeValues();
    displayDesires();
    sendCommands();
    
  }
  /***********************************************************************************************/

  else if (mode == 3){
   displayDesires();
    displayReadings();
    sendCommands();
  }
  /***********************************************************************************************/

  else if (mode == 4){
    displayDesires();
    displayReadings();
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
  
//  if (currentMillis - lastDisplayTime > 500){
//    lastDisplayTime = currentMillis;
//    Serial.print("Switch States: Button = ");
//    Serial.print(pushButtonState);
//    Serial.print(", Left = ");
//    Serial.print(leftButtonState);
//    Serial.print(", Right = ");
//    Serial.print(rightButtonState);
//    Serial.print(", Up = ");
//    Serial.print(upButtonState);
//    Serial.print(", Down = ");
//    Serial.println(downButtonState);
//  }
  
  //loopCount+=1;
  //Serial.println(loopCount);
}
/***********************************************************************************************/
/***********************************************************************************************/
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
  speedSetting = 0;
  memset(differenceList,0,sizeof(differenceList)); // clears the integrator
}
/***********************************************************************************************/
/***********************************************************************************************/
void sendCommands(){
  if (currentMillis - lastSentTime > 199){
    lastSentTime = currentMillis;
  
    Serial.print(mode);
    Serial.print(":");
    Serial.print(modeNames[mode]);
  
    Serial.print("\tSpd:");
    Serial.print(speedSetting);
    
    Serial.print("\tHead:");
    Serial.print(currentHeading);
    
    Serial.print("\tFinal:");
    Serial.print(finalHeading);
    
    Serial.print("\tGoal:");
    Serial.print(goalSetting);
    
    Serial.print("\tDiff:");
    Serial.print(difference);
    
    Serial.print("\tSum:");
    Serial.print(sum);
              
    Serial.print("\tDrift:");
    Serial.print(drift);
         
    Serial.print("\tLeft:");
    Serial.print(leftMotor);
    
    Serial.print("\tRight:");
    Serial.println(rightMotor);
    
    motorMessage[0]=byte(rightMotor);
    motorMessage[1]=byte(leftMotor);
    CAN0.sendMsgBuf(0x31A, 0, 2, motorMessage );
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
    goalSetting +=1;
    if (goalSetting > 360) goalSetting = 1;
  }
}
/***********************************************************************************************/
/***********************************************************************************************/
void decrementGoal(){
  if (currentMillis - goalChangeTimer > goalChangeDelay){
    goalChangeTimer=currentMillis;
    goalSetting -=1;
    if (goalSetting < 0) goalSetting = 359;
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
     heading1 = (rxBuf[0]*256 + rxBuf[1])/10.0;
   }
   else if (rxId == 0x43e){
     headingReading = (rxBuf[0]*256 + rxBuf[1])/10.0;
     gpsSpeed = (rxBuf[2]*256 + rxBuf[3])*1.15078;
     gpsAngle = (rxBuf[4]*256 + rxBuf[5]);
     gpsSats  = rxBuf[6];
     gpsFix   = rxBuf[7];
   }
  
  currentHeading = int(headingReading + headingFixedOffset);
  
  if (currentHeading > 360) currentHeading -= 360;
  if (currentHeading < 0) currentHeading += 360;
  
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
    sprintf(displayBuffer,"S:%3i G:%3i", speedSetting,goalSetting);
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
    sprintf(displayBuffer,"N:%1i S:%2i H:%3i", gpsSats,gpsSpeed,currentHeading);
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
  if (currentMillis - lastCalculateTime > 50){
        lastCalculateTime = currentMillis;
        difference = goalSetting - currentHeading;;
        if (difference <= -180)  difference += 360;
        else if (difference > 180)  difference -= 360;

        differenceList[diffIndex] = difference;
        diffIndex+=1;
        if (diffIndex >= memorySize) diffIndex = 0;
        
        sum = 0;
        for (int j = 0; j < memorySize; j++){
          sum += differenceList[j];
        } 
        
        drift = sum >> 6; //This is like division 

        int tempRightMotor = speedSetting - K*difference - zeroAdjust - I*drift + 100;
        int tempLeftMotor  = speedSetting + K*difference + zeroAdjust + I*drift + 100;
        rightMotor = constrain(tempRightMotor,0,200);
        leftMotor  = constrain(tempLeftMotor,0,200);
  }
        
}
/***********************************************************************************************/
/***********************************************************************************************/

