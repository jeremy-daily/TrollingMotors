#include <SerLCD.h>
#include <FlexCAN.h>
#include <Bounce2.h>


SerLCD lcd;

CAN_message_t joyMessage;
CAN_message_t msg;

#define DEBOUNCE_DELAY 25

//Define the button input pins
#define rightButton 9
#define downButton  16
#define pushButton  12
#define leftButton  17
#define upButton    15
#define redButton   11
#define greenButton 10

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


char str11[9];
char str12[9];
char str21[9];
char str22[9];

uint32_t currentMillis;
uint32_t changeModeMillis;
uint8_t numModes;
uint8_t mode;

void setup() {
  Serial1.begin(9600);
  Serial.println("It's time to go fishing with the Dailys!!");
  //start CAN communications
  Can0.begin(500000);
  
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
  
  rightDebouncer.interval(DEBOUNCE_DELAY);
  leftDebouncer.interval(DEBOUNCE_DELAY);
  downDebouncer.interval(DEBOUNCE_DELAY);
  pushDebouncer.interval(DEBOUNCE_DELAY);
  upDebouncer.interval(DEBOUNCE_DELAY);
  redDebouncer.interval(DEBOUNCE_DELAY);
  greenDebouncer.interval(DEBOUNCE_DELAY); // interval in ms

  lcd.begin(Serial1);
  lcd.setBacklight(255, 255, 255); //Set backlight to bright white
  lcd.setContrast(7); //Set contrast. Lower to 0 for higher contrast.
  delay(1000);
  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  delay(100);
  lcd.print("Let's go fishing");
  lcd.print("Fun for everyone");

  delay(2000);
  joyMessage.id = 0x700;
  joyMessage.len = 1;

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


  /* 
  lcd.write(254); //escape character
  lcd.write(206); //Move Cursor to the bottom last point on a 16x2 display  

  //Calculate a new symbol for button combinations
  //Space is 0x20 or 32
  char buttonSymbol = 32 + 1*!pushButtonState + 2*!greenButtonState + 4*!redButtonState;
  lcd.print(buttonSymbol);
  
  //Automatically move to the next slot and print Joystick positions.
  
  //lcd.write(254); //escape character
  //lcd.write(207); //Move Cursor to the bottom last point on a 16x2 display  
  if (!rightButtonState) lcd.print("R");
  else if (!leftButtonState) lcd.print("L");
  else if (!downButtonState) lcd.print("D");
  else if (!upButtonState) lcd.print("U");
  else lcd.print(" ");
  */
   
  readCANbus();
  
  if (currentMillis - lastRXTime > 1200){
    if (!writeOnce){
      lcd.setCursor(0, 0);
      lcd.print("Lost CAN Comms. ");
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
    bitWrite(joyMessage.buf[0],0,!upButtonState);
    bitWrite(joyMessage.buf[0],1,!downButtonState);
    bitWrite(joyMessage.buf[0],2,!leftButtonState);
    bitWrite(joyMessage.buf[0],3,!rightButtonState);
    bitWrite(joyMessage.buf[0],4,!pushButtonState);
    bitWrite(joyMessage.buf[0],5,doubleClick);
    bitWrite(joyMessage.buf[0],6,!redButtonState);
    bitWrite(joyMessage.buf[0],7,!greenButtonState);

    Can0.write(joyMessage);
    Serial.println(joyMessage.buf[0],BIN);
  }
}
/***********************************************************************************************/
/***********************************************************************************************/


/***********************************************************************************************/
/***********************************************************************************************/

void readCANbus(){
  Can0.read(msg); // Read data: len = data length, buf = data byte(s)
  byte i = 0;
  if (msg.id == 0x210){
    numModes = msg.buf[0];
    //mode = msg.buf[1];
    lastRXTime = currentMillis;
    writeOnce=false;
  }
  else if (msg.id == 0x211){ //Display Characters on first quarter of screen
    lcd.write(254); //escape character
    lcd.write(128); //Move Cursor
    for (i=0;i<8;i++) str11[i] = constrain(msg.buf[i],32,126);
    lcd.print(str11);   
  }
  else if (msg.id == 0x212){
    lcd.write(254); //escape character
    lcd.write(136); //Move 
    for (i=0;i<8;i++) str12[i] = constrain(msg.buf[i],32,126);
    lcd.print(str12);
  }
  else if (msg.id == 0x221){
    lcd.write(254); //escape character
    lcd.write(192); //Move 
    for (i=0;i<8;i++) str21[i] = constrain(msg.buf[i],32,126);
    lcd.print(str21);
  }
  else if (msg.id == 0x222){
    lcd.write(254); //escape character
    lcd.write(200); //Move 
    for (i = 0; i < 8 ; i++) str22[i] = constrain(msg.buf[i],32,126);
    lcd.print(str22);
  }
  else if (msg.id == 0x221){
    lcd.write(254); //escape character
    lcd.write(192); //Move 
    for (i=0;i<8;i++) str21[i] = constrain(msg.buf[i],32,126);
    lcd.print(str21);
    }
  else if (msg.id == 0x222){
    lcd.write(254); //escape character
    lcd.write(200); //Move 
    for (i = 0; i < 8 ; i++) str22[i] = constrain(msg.buf[i],32,126);
    lcd.print(str22);
  }
  msg.id = 0;
}
