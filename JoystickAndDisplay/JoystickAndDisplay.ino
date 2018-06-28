#include <SoftwareSerial.h>
#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <Bounce2.h>

SoftwareSerial dispSerial(8,9);

// enable the CAN interface with the MCP2515 chip
MCP_CAN CAN0(10); 

byte joyMessage[8];

char numModes = 5;

//CAN interface messages (Borrowed from the example).
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

//Define the button input pins
#define rightButton 14
#define downButton  15
#define pushButton  17
#define leftButton  5
#define upButton    3
#define redButton   8
#define greenButton 7

Bounce rightDebouncer = Bounce(); 
Bounce downDebouncer  = Bounce(); 
Bounce pushDebouncer  = Bounce(); 
Bounce leftDebouncer  = Bounce(); 
Bounce upDebouncer    = Bounce(); 
Bounce redDebouncer   = Bounce(); 
Bounce greenDebouncer = Bounce(); 

bool rightButtonState;
bool downButtonState;
bool pushButtonState;
bool leftButtonState;
bool upButtonState;
bool redButtonState;
bool greenButtonState;

unsigned long lastJoyTime = 0;
unsigned long lastRXTime = 0;

boolean writeOnce   = false;
boolean doubleClick = false;
boolean changeModeOK = true;

char str11[9];
char str12[9];
char str21[9];
char str22[9];

uint32_t currentMillis;
uint32_t changeModeMillis;

uint8_t mode;

void setup() {

  Serial.begin(115200);
  Serial.println("It's time to go fishing with the Dailys!!");
  //start CAN communications
  Serial.println("Setting up CAN0...");
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  CAN0.init_Mask(0,1,0x7FF);
  CAN0.init_Mask(2,1,0x7FF);
  
  CAN0.init_Filt(0,1,0x211 ^ 0x7FF);
  CAN0.init_Filt(1,1,0x212 ^ 0x7FF);
  CAN0.init_Filt(2,1,0x221 ^ 0x7FF);
  CAN0.init_Filt(3,1,0x222 ^ 0x7FF);
  CAN0.init_Filt(4,1,0x210 ^ 0x7FF);

  // make the pushbutton's pin an input:
  pinMode(rightButton, INPUT_PULLUP);
  pinMode(leftButton,  INPUT_PULLUP);
  pinMode(downButton,  INPUT_PULLUP);
  pinMode(pushButton,  INPUT_PULLUP);
  pinMode(upButton,    INPUT_PULLUP);
  pinMode(redButton,   INPUT_PULLUP);
  pinMode(greenButton, INPUT_PULLUP);

  rightDebouncer.attach(rightButton);
  leftDebouncer.attach(leftButton);
  downDebouncer.attach(downButton);
  pushDebouncer.attach(pushButton);
  upDebouncer.attach(upButton);
  redDebouncer.attach(redButton);
  greenDebouncer.attach(greenButton);
  
  rightDebouncer.interval(15);
  leftDebouncer.interval(15);
  downDebouncer.interval(15);
  pushDebouncer.interval(15);
  upDebouncer.interval(15);
  redDebouncer.interval(15);
  greenDebouncer.interval(15); // interval in ms

  dispSerial.begin(9600);
  delay(500);

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
  delay(2000);

  dispSerial.write(254);
  dispSerial.write(1); //clear screen

}

// the loop routine runs over and over again forever:
void loop() {
  currentMillis = millis();

  rightDebouncer.update();
  leftDebouncer.update();
  downDebouncer.update();
  pushDebouncer.update();
  upDebouncer.update();
  redDebouncer.update();
  greenDebouncer.update();

  rightButtonState = rightDebouncer.read();
  leftButtonState = leftDebouncer.read();
  downButtonState = downDebouncer.read();
  pushButtonState = pushDebouncer.read();
  upButtonState = upDebouncer.read();
  redButtonState = redDebouncer.read();
  greenButtonState = greenDebouncer.read();

  

  //Calculate a new symbol for button combinations
  //Space is 0x20 or 32

  if (!redButtonState){
    mode = 0;
    dispSerial.write(254); // move cursor to beginning of second line (254, 192)
    dispSerial.write(192);
    dispSerial.print("              ");
    dispSerial.write(254); // move cursor to beginning of second line (254, 192)
    dispSerial.write(192);
    dispSerial.print("Reset: Mode ");
    dispSerial.print(mode,DEC);
  }
  
  if (!greenButtonState && !pushButtonState && changeModeOK){
 
    if (!upButtonState) {
      mode++;
      if (mode > numModes) mode = 0;
      changeModeOK = false; 
      changeModeMillis = currentMillis;
    }
    if (!downButtonState) {
      mode--;
      if (mode > numModes) mode = numModes;
      changeModeOK = false;
      changeModeMillis = currentMillis;  
    }
    dispSerial.write(254); // move cursor to beginning of first line (254, 128)
    dispSerial.write(192);
    dispSerial.print("              ");
    dispSerial.write(254); // move cursor to beginning of first line (254, 128)
    dispSerial.write(192);
    dispSerial.print("Mode ");
    dispSerial.print(mode,DEC);
    writeOnce=false;
  }

  if (currentMillis - changeModeMillis > 400){
    changeModeOK = true;
  }

  dispSerial.write(254); //escape character
  dispSerial.write(206); //Move Cursor to the bottom next to last point on a 16x2 display  
  char buttonSymbol = 32 + 1*!pushButtonState + 2*!greenButtonState + 4*!redButtonState;
  dispSerial.print(buttonSymbol);
  
  //Automatically move to the next slot and print Joystick positions.
  
  //dispSerial.write(254); //escape character
  //dispSerial.write(207); //Move Cursor to the bottom last point on a 16x2 display  
  if (!rightButtonState) dispSerial.print("R");
  else if (!leftButtonState) dispSerial.print("L");
  else if (!downButtonState) dispSerial.print("D");
  else if (!upButtonState) dispSerial.print("U");
  else dispSerial.print(" ");

   
  readCANbus();
  
  if (currentMillis - lastRXTime > 1200){
    if (!writeOnce){
      dispSerial.write(254); // move cursor to beginning of first line (254, 128)
      dispSerial.write(128);
      dispSerial.print("                ");
      dispSerial.write(254); // move cursor to beginning of first line (254, 128)
      dispSerial.write(128);
      dispSerial.print("Lost CAN Comms.");
      writeOnce=true;
    }
  }
 
  sendJoyStick();
}


/***********************************************************************************************/
/***********************************************************************************************/
void sendJoyStick(){
  if (currentMillis - lastJoyTime >= 50){
    lastJoyTime = currentMillis;
    joyMessage[0] = mode;
    bitWrite(joyMessage[1],0,!upButtonState);
    bitWrite(joyMessage[1],1,!downButtonState);
    bitWrite(joyMessage[1],2,!leftButtonState);
    bitWrite(joyMessage[1],3,!rightButtonState);
    bitWrite(joyMessage[1],4,!pushButtonState);
    bitWrite(joyMessage[1],5,doubleClick);
    bitWrite(joyMessage[1],6,!redButtonState);
    bitWrite(joyMessage[1],7,!greenButtonState);
    
    CAN0.sendMsgBuf(0x700, 0, 2, joyMessage );
  }
}
/***********************************************************************************************/
/***********************************************************************************************/


/***********************************************************************************************/
/***********************************************************************************************/

void readCANbus(){
  if(CAN0.checkReceive()){
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)

    if (rxId == 0x210){
      numModes=rxBuf[0];
      //mode = rxBuf[1];
      lastRXTime = currentMillis;
    }
   
    uint8_t i = 0; 
    if (rxId == 0x211){ //Display Characters on first quarter of screen
      dispSerial.write(254); //escape character
      dispSerial.write(128); //Move Cursor
      for (i=0;i<8;i++) str11[i] = constrain(rxBuf[i],32,126);
      dispSerial.print(str11);
      
      lastRXTime = currentMillis;
    }
    else if (rxId == 0x212){
      dispSerial.write(254); //escape character
      dispSerial.write(136); //Move 
      for (i=0;i<8;i++) str12[i] = constrain(rxBuf[i],32,126);
      dispSerial.print(str12);
      lastRXTime = currentMillis;
    }
    else if (rxId == 0x221){
      dispSerial.write(254); //escape character
      dispSerial.write(192); //Move 
      for (i=0;i<8;i++) str21[i] = constrain(rxBuf[i],32,126);
      dispSerial.print(str21);
      lastRXTime = currentMillis;
    }
    else if (rxId == 0x222){
      dispSerial.write(254); //escape character
      dispSerial.write(200); //Move 
      for (i = 0; i < 8 ; i++) str22[i] = constrain(rxBuf[i],32,126);
      dispSerial.print(str22);
      Serial.print(str11);
      Serial.print(" ");
      Serial.print(str12);
      Serial.print(" ");
      Serial.print(str21);      
      Serial.print(" ");
      Serial.println(str22);
      writeOnce=true;
      lastRXTime = currentMillis;
    }
  }
}

