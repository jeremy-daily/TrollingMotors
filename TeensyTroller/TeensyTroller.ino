#include <TinyGPS++.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include "SPI.h"
#include "ILI9341_t3.h"
#include  <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SFE_HMC6343.h>
#include <Servo.h> 

double turnRate = 0.01;

double speedK = 1;
double speedI = 1;
double speedD = 0;

double turnK = 1;
double turnI = 1;
double turnD = 0;

double angleK = 1;
double angleI = 1;
double angleD = 0;


IntervalTimer calculateMotorOutputTimer; // this is interrupt based

elapsedMillis broadcastCANtimer; //set up intervals 
elapsedMillis waitingForCANtimer;
elapsedMillis printTFTtimer;
elapsedMillis debugSerialtimer;
elapsedMillis mode0displaytimer;
elapsedMillis mode1displaytimer;
elapsedMillis mode2displaytimer;
elapsedMillis mode3displaytimer;
elapsedMillis mode4displaytimer;
elapsedMillis mode5displaytimer;
elapsedMillis mode6displaytimer;
elapsedMillis CANaliveTimer;
elapsedMillis speedSettingTimer;
elapsedMillis broadcastCANmodeTimer;


const int speedSetTime = 50; //set how quickly the speed changes.

boolean mode1started = false;
boolean mode2started = false;
boolean mode3started = false;
boolean mode4started = false;
boolean mode5started = false;

byte mode = 0; 
byte currentMode = 0;
byte numberOfModes = 6; //This limits the number of displayed modes.
//char modeNames[7][6]={" Off ","Man. ","TurnL","TurnR","Fix  ","Fig8 ", "Tune "}; // This array is the length of the number of m

boolean rightButtonState = LOW;
boolean leftButtonState = LOW;
boolean downButtonState = LOW;
boolean pushButtonState = LOW;
boolean upButtonState = LOW;


double fixPointLat = 48.85826;
double fixPointLon = 2.294516;
double distanceToFixPoint = 0;
double courseToFixPoint = 0;
double speedGoal=0;
double coarseGoal=0;

int speedSetting;
 
//Initialize the GPS
TinyGPSPlus gps;


// Setup the TFT display.
#define TFT_DC 20
#define TFT_CS 21
// initialize the display
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

SFE_HMC6343 compass; // Declare the sensor object

Servo rightServo;  // create servo object to control a servo 
Servo leftServo;  // create servo object to control a servo 

int rightVal;    // variable to read the value from the analog pin 
int leftVal;    // variable to read the value from the analog pin 

//Set up CAN messaging
FlexCAN CANbus(500000);
static CAN_message_t txmsg,rxmsg;
uint32_t CANTXcount = 0;
uint32_t CANRXcount = 0;
uint32_t ID = 0;
char message[9] ="        "; //initialize with spaces

// setup the IMU sensor
Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> euler;
imu::Vector<3> gyro;

float yawAngle = 0.0;
float yawRate = 0.0;
float compassHeading = 0.0;

void setup() {
  
  Serial.begin(115200); //debug console
  
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(3);
  tft.print("Setting Up...");
  
  tft.println("Starting IMU");
   /* Initialise the sensor */
//  Wire.begin();
//  Wire.beginTransmission(0x28);
//  Wire.write(0x3F);
//  Wire.write(0x20);
//  Wire.endTransmission();
  bno.begin();
  bno.setExtCrystalUse(true);
  
  tft.print("Starting Srvo");
  rightServo.attach(23);  // attaches the servo on pin 23 to the servo object 
  leftServo.attach(16);  // attaches the servo on pin 16 to the servo object 
  
  
  tft.println("Starting GPS");
  Serial1.begin(9600);
 
  tft.print("Starting Comp");
  compass.init();

  tft.println("Starting CAN");
  delay(100);
  CANbus.begin();
  
  calculateMotorOutputTimer.begin(calculateMotorOutput, 50000); //call the sendCANmessages every 0.05 seconds

  
  strncpy(message,"Fishing ",8);
  txmsg.id=0x211; //Send to the upper left
  txmsg.len=8;
  for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
  CANbus.write(txmsg);
  delay(50);
  
  strncpy(message,"is great",8);
  txmsg.id=0x212; //sent to the upper right
  txmsg.len=8;
  for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
  CANbus.write(txmsg);
  delay(50);
  
  strncpy(message,"today. H",8);
  txmsg.id=0x221; //sent to the lower left
  txmsg.len=8;
  for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
  CANbus.write(txmsg);
  delay(50);
    
  strncpy(message,"ave fun.",8);
  txmsg.id=0x222; //sent to the lower right
  txmsg.len=8;
  for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
  CANbus.write(txmsg);
  delay(50);
  
   
  tft.print("CAN msgs sent");

 
  delay(1000);
  
  tft.fillScreen(ILI9341_BLACK);
} 

void sendCANmessages(){
  if (broadcastCANtimer >= 1000) {
    broadcastCANtimer = 0;
  
    //GPS Messages
    if (gps.location.isUpdated() ){
      txmsg.id=0x43e;
      txmsg.len=8;
      
      txmsg.buf[0]=byte( (gps.speed.value() & 0xFF000000) >> 24);
      txmsg.buf[1]=byte( (gps.speed.value() & 0x00FF0000) >> 16);
      txmsg.buf[2]=byte( (gps.speed.value() & 0x0000FF00) >>  8);
      txmsg.buf[3]=byte( (gps.speed.value() & 0x000000FF) >>  0);
      txmsg.buf[4]=byte( (gps.course.value() & 0xFF000000) >> 24);
      txmsg.buf[5]=byte( (gps.course.value() & 0x00FF0000) >> 16);
      txmsg.buf[6]=byte( (gps.course.value() & 0x0000FF00) >>  8);
      txmsg.buf[7]=byte( (gps.course.value() & 0x000000FF) >>  0);
      
      CANbus.write(txmsg);
      CANTXcount++;
    }
    
    txmsg.id=0x210;
    txmsg.len=8;
    
    txmsg.buf[0]=mode;
    txmsg.buf[1]=numberOfModes;
    txmsg.buf[2]=0xFF;
    txmsg.buf[3]=0xFF;
    txmsg.buf[4]=0xFF;
    txmsg.buf[5]=0xFF;
    txmsg.buf[6]=0xFF;
    txmsg.buf[7]=0xFF;
    
    CANbus.write(txmsg);
    CANTXcount++;
    }
}
  


void readCANmessages(){
  while ( CANbus.read(rxmsg) ) {
      waitingForCANtimer = 0; //reset the can message timeout
      CANRXcount++;
      ID = rxmsg.id;
      if (ID == 0x007){
        CANaliveTimer = 0;
        mode=rxmsg.buf[0];
        upButtonState=bitRead(rxmsg.buf[1],0);
        downButtonState=bitRead(rxmsg.buf[1],1);
        leftButtonState=bitRead(rxmsg.buf[1],2);
        rightButtonState=bitRead(rxmsg.buf[1],3);
        pushButtonState=bitRead(rxmsg.buf[1],4);
      }
  }
}
void displayData(){
  if (printTFTtimer > 250){
    printTFTtimer = 0;
    //tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(70,0);
    tft.print("Compass");
    
    tft.setCursor(0,30);
    tft.print("Heading: ");
    int xStart = 150;
    int yLine = 1;
    tft.fillRect(xStart, 30*yLine, 240-xStart, 30, ILI9341_BLACK);
    tft.setCursor(150,30);
    tft.print(int(compass.heading/10));

    tft.setCursor(0,60);
    tft.print("Pitch: ");
    xStart = 150;
    yLine = 2;
    tft.fillRect(xStart, 30*yLine, 240-xStart, 30, ILI9341_BLACK);
    tft.setCursor(150,60);
    //tft.print(int16_t(compass.pitch/10.0),DEC);
    int pitch = compass.pitch;
    if (pitch > 32768) pitch = pitch - 65536;
    tft.print(pitch/10,DEC);

    tft.setCursor(0,90);
    tft.print("Roll: ");
    xStart = 150;
    yLine = 3;
    tft.fillRect(xStart, 30*yLine, 240-xStart, 30, ILI9341_BLACK);
    tft.setCursor(150,90);
    int roll = compass.roll;
    if (roll > 32768) roll = roll - 65536;
    tft.print(roll/10);

    tft.setCursor(90,120);
    tft.print("IMU");
    
    tft.setCursor(0,150);
    tft.print("Yaw: ");
    xStart = 110;
    yLine = 5;
    tft.fillRect(xStart, 30*yLine, 240-xStart, 30, ILI9341_BLACK);
    tft.setCursor(110,150);
    
    tft.print(yawAngle);
    
    tft.setCursor(0,180);
    tft.print("Pitch: ");
    xStart = 110;
    yLine = 6;
    tft.fillRect(xStart, 30*yLine, 240-xStart, 30, ILI9341_BLACK);
    tft.setCursor(110,180);
    float pitchIMU = euler.z();
    tft.print(pitchIMU);

    
    tft.setCursor(0,210);
    tft.print("Roll: ");
    xStart = 110;
    yLine = 7;
    tft.fillRect(xStart, 30*yLine, 240-xStart, 30, ILI9341_BLACK);
    tft.setCursor(110,210);
    float rollIMU = euler.y();
    tft.print(rollIMU);

    tft.setCursor(0,240);
    tft.print("Rate:");
    xStart = 130;
    yLine = 8;
    tft.fillRect(xStart, 30*yLine, 240-xStart, 30, ILI9341_BLACK);
    tft.setCursor(xStart,240);
    tft.print(yawRate);

    if (pushButtonState) tft.fillRect(0, 270, 240, 5, ILI9341_WHITE);
    else tft.fillRect(0, 270, 240, 5, ILI9341_BLACK);

    
    
    if (downButtonState) tft.fillRect(0, 280, 240, 30, ILI9341_WHITE);
    else
    {
      tft.fillRect(0, 280, 240, 30, ILI9341_BLACK);
      tft.setCursor(0,280);
      tft.print("Sats:");
      tft.setCursor(90,280);
      tft.print(gps.satellites.value());
      tft.print(" Mode:");
      tft.print(mode);
      
    }
  }
}

void debugData(){
  if (debugSerialtimer >= 200){
    debugSerialtimer = 0;
    Serial.print(mode);
    Serial.print("\t");
    Serial.print(speedGoal);
    Serial.print("\t");
    Serial.print(gps.speed.mph());
    Serial.print("\t");
    Serial.print(coarseGoal);
    Serial.print("\t");
    Serial.print(gps.course.deg());
    Serial.print("\t");
    Serial.print(compassHeading);
    Serial.print("\t");
    Serial.print(yawAngle);
    Serial.print("\t");
    Serial.print(yawRate,6);
    Serial.print("\t");
    Serial.print(gps.satellites.value());
    Serial.print("\t");
    Serial.print(gps.altitude.feet());
    Serial.print("\t");
    Serial.print(gps.location.lat(),8);
    Serial.print("\t");
    Serial.print(gps.location.lng(),8); 
    Serial.print("\t");
    Serial.print(fixPointLat,8);
    Serial.print("\t");
    Serial.print(fixPointLon,8);
    Serial.print("\t");
    Serial.print(distanceToFixPoint);
    Serial.print("\t");
    Serial.print(courseToFixPoint);
    Serial.print("\t");
    
    Serial.println(CANTXcount);
  }
}


void calculateMotorOutput(){
  if (mode == 1){
    rightVal = map(rightVal, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  
  }
  else {
    leftServo.write(90);
    rightServo.write(90);
  }
  
}

 
void loop() {
  if (CANaliveTimer > 500) mode = 0;
  
  //measure stuff
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yawAngle = euler.x();
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  yawRate = gyro.z();
  compass.readHeading();
  compassHeading = compass.heading/10.0;
  while (Serial1.available())
     gps.encode(Serial1.read());
  
  //send stuff
  displayData();
  debugData();
  sendCANmessages();

  
  //get user input
  readCANmessages();
  
  if (mode != currentMode) resetOutputs();
  
  if (mode == 0){
    displayMode0();
    
    
  }
  else if (mode == 1){
    displayMode1();
   
    if (mode1started){
      if (speedSettingTimer > speedSetTime) {
        speedSettingTimer = 0;
        if (upButtonState) speedSetting++;
        if (downButtonState) speedSetting--;
        speedSetting = constrain(speedSetting,-99,100);
      }
    }

    if (upButtonState && pushButtonState) {mode1started = true; speedSetting=0;}
    if (downButtonState && pushButtonState){ speedSetting=0; }

  }
  else if (mode == 2){
    displayMode2();
    if (upButtonState && pushButtonState) mode2started = true;
    
    distanceToFixPoint = gps.distanceBetween(gps.location.lat(), gps.location.lng(), fixPointLat, fixPointLon);
    courseToFixPoint = gps.courseTo(gps.location.lat(), gps.location.lng(), fixPointLat, fixPointLon);
 
  }
  else if (mode == 3){
    if (upButtonState && pushButtonState) mode3started = true;
    displayMode3();
    
  }
  else if (mode == 4){
    if (upButtonState && pushButtonState) mode4started = true;
    displayMode4();
    
  }
  else if (mode == 5){
    if (upButtonState && pushButtonState) mode5started = true;
    displayMode5();
    
  }
  else{
    mode = 0;
  }
    
}

void resetOutputs(){
  speedSetting = 0;
  mode1started = false;
  mode2started = false;
  mode3started = false;
  mode4started = false;
  mode5started = false;
  
  tft.fillScreen(ILI9341_GREEN);
  delay(100);
  tft.fillScreen(ILI9341_BLACK);
  currentMode = mode;
}

void displayMode0(){
  if (mode0displaytimer >= 200){
    mode0displaytimer = 0;
    
    sprintf(message,"%i  OFF  ",mode);
    txmsg.id=0x211; //sent to the lower right
    txmsg.len=8;
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;
    
    sprintf(message," Sats:%2i ",int(gps.satellites.value()));
    txmsg.id=0x212; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message,"H:%3i Y:",int(gps.course.deg()));
    txmsg.id=0x221; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message,"%3i S:%2i",int(yawAngle),int(gps.speed.mph()) );
    txmsg.id=0x222; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;
  }
}


void displayMode1(){
  if (mode1displaytimer >= 200){
    mode1displaytimer = 0;
    
    sprintf(message,"%i Manual",mode);
    txmsg.id=0x211; //sent to the lower right
    txmsg.len=8;
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message," Spd:%3i",int(speedSetting));
    txmsg.id=0x212; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    if (mode1started){
      sprintf(message,"H:%3i Y:",int(gps.course.deg()));
      txmsg.id=0x221; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
  
      sprintf(message,"%3i S:%2i",int(yawAngle),int(gps.speed.mph()) );
      txmsg.id=0x222; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message,"Butn+Up ",8);
      txmsg.id=0x221; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
  
      strncpy(message,"to Start",8);
      txmsg.id=0x222; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++; 
    }
  }
}


void displayMode2(){
  if (mode2displaytimer >= 200){
    mode2displaytimer = 0;
    
    sprintf(message,"%i Turn90",mode);
    txmsg.id=0x211; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message," Sats:%2i  ",int(gps.satellites.value()));
    txmsg.id=0x212; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;
    
    if (mode2started){
      sprintf(message,"H:%3i Y:",int(gps.course.deg()));
      txmsg.id=0x221; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
  
      sprintf(message,"%3i S:%2i",int(yawAngle),int(gps.speed.mph()) );
      txmsg.id=0x222; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message,"Butn+Up ",8);
      txmsg.id=0x221; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
  
      strncpy(message,"to Start",8);
      txmsg.id=0x222; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++; 
    }

    
  }
}


void displayMode3(){
  if (mode3displaytimer >= 200){
    mode3displaytimer = 0;
    
    sprintf(message,"%i Anchor",mode);
    txmsg.id=0x211; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message," Sats:%2i  ",int(gps.satellites.value()));
    txmsg.id=0x212; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    if (mode3started){
      sprintf(message,"H:%3i Y:",int(gps.course.deg()));
      txmsg.id=0x221; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
  
      sprintf(message,"%3i S:%2i",int(yawAngle),int(gps.speed.mph()) );
      txmsg.id=0x222; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message,"Butn+Up ",8);
      txmsg.id=0x221; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
  
      strncpy(message,"to Start",8);
      txmsg.id=0x222; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++; 
    }

    
  }
}


void displayMode4(){
  if (mode4displaytimer >= 200){
    mode4displaytimer = 0;
    
    mode1displaytimer = 0;
    sprintf(message,"%i Fig. 8",mode);
    txmsg.id=0x211; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message," Sats:%2i",int(gps.satellites.value()));
    txmsg.id=0x212; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;
    
    if (mode4started){
      sprintf(message,"H:%3i Y:",int(gps.course.deg()));
      txmsg.id=0x221; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
  
      sprintf(message,"%3i S:%2i",int(yawAngle),int(gps.speed.mph()) );
      txmsg.id=0x222; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message,"Butn+Up ",8);
      txmsg.id=0x221; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
  
      strncpy(message,"to Start",8);
      txmsg.id=0x222; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++; 
    }

    
  }
}


void displayMode5(){
  if (mode5displaytimer >= 200){
    mode5displaytimer = 0;
    
    mode1displaytimer = 0;
    sprintf(message,"%i Tune  ",mode);
    txmsg.id=0x211; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message," Sats:%2i  ",int(gps.satellites.value()));
    txmsg.id=0x212; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    if (mode5started){
      sprintf(message,"H:%3i Y:",int(gps.course.deg()));
      txmsg.id=0x221; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
  
      sprintf(message,"%3i S:%2i",int(yawAngle),int(gps.speed.mph()) );
      txmsg.id=0x222; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message,"Butn+Up ",8);
      txmsg.id=0x221; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
  
      strncpy(message,"to Start",8);
      txmsg.id=0x222; //sent to the lower right
      for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++; 
    }
    
  }
}

