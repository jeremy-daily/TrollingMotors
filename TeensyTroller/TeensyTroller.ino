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


const int memorySize = 512;
double differenceList[512];
double differenceSpeedList[512];
const uint32_t deltaT = 50000; //usec for calculations and output

double yawOffset = 0;
double compassOffset = 0;

double turnRate = 0.01;

double speedK = .1;
double speedI = .01;
double speedD = 0;

double turnK = 100;
double turnI = 0;
double turnD = 0;

double angleK = 1;
double angleI = .01;
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
elapsedMillis courseSettingTimer;

const int speedSetTime = 150; //set how quickly the speed changes.
const int courseSetTime = 150; //set how quickly the speed changes.

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

int diffIndex = 0;
int diffSpeedIndex = 0;

double difference = 0;
double speedDifference = 0;
double currentHeading = 0;
double goalAngle = 0;
double goalSpeed = 0;
double turnSetting = 0;

int speedSetting;
int angleSetting;

int leftMotor = 90;
int rightMotor = 90;
 
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
  
  calculateMotorOutputTimer.begin(calculateMotorOutput, deltaT); //call the sendCANmessages every 0.05 seconds

  
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

  compass.readHeading();
  yawOffset = compass.heading/10.0 - compassOffset;
 
  delay(1000);
  
  displayTemplate();
} 

void resetCompassOffset(){
 if (gps.speed.mph() > 1.0 && abs(yawRate) < 0.1 ) //compass heading can be realigned to velocity vector
  {
    if (gps.course.deg() >= 270 && compass.heading/10.0 <= 90)
    {
      compassOffset = gps.course.deg() - compass.heading/10.0 + 360;
    }
    else if (gps.course.deg() <= 90 && compass.heading/10.0 >= 270)
    {
      compassOffset = gps.course.deg() - compass.heading/10.0 - 360;
    }
    else
    {
      compassOffset = gps.course.deg() - compass.heading/10.0;
    }
    
  }
  else 
  {
    compassOffset = 0; // this is the constant angle of misalignment of the sensor and the boat.
  }
}

void resetYawOffset(){
  currentHeading = getCompassHeading();
  
  if (currentHeading >= 270 && euler.x() <= 90)
    {
      yawOffset = euler.x() - currentHeading -  + 360;
    }
    else if (currentHeading <= 90 && euler.x() >= 270)
    {
      yawOffset = euler.x() - currentHeading - 360;
    }
    else
    {
      yawOffset = euler.x() - currentHeading;
    }
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
  }
  
  if (broadcastCANmodeTimer >= 100) {
    broadcastCANmodeTimer = 0;
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
      if (ID == 0x700){
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


void displayTemplate(){
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0,0);
  tft.print("Mode:X Sat:XX");
  
  tft.setCursor(0,30);
  tft.print("Comp Head:XXX");
  
  tft.setCursor(0,60);
  tft.print("Yaw Angle:XXX");

  tft.setCursor(0,90);
  tft.print("GPS Headn:XXX");

  tft.setCursor(0,120);
  tft.print("GoalAngle:XXX");
  
  tft.setCursor(0,150);
  tft.print("GPSSpeed:XX.X");

  tft.setCursor(0,180);
  tft.print("GoalSpeed:X.X");

  tft.setCursor(0,210);
  tft.print("LeftMotor:XXX");
  
  tft.setCursor(0,240);
  tft.print("RghtMotor:XXX");

  tft.setCursor(0,270);
  tft.print("YawRate:XXX.X");

  
}

void displayData(){
  if (printTFTtimer > 250){
    printTFTtimer = 0;

    if (downButtonState) tft.fillRect(0, 310, 240, 10, ILI9341_WHITE);
    else tft.fillRect(0, 310, 240, 30, ILI9341_BLACK);
      //    if (upButtonState) tft.fillRect(0, 0, 240, 10, ILI9341_WHITE);
      //    else tft.fillRect(0, 0, 240, 10, ILI9341_BLACK);
      //    if (rightButtonState) tft.fillRect(230, 0, 10, 320, ILI9341_RED);
      //    else tft.fillRect(230, 0, 10, 320, ILI9341_BLACK);
      //    if (leftButtonState) tft.fillRect(0, 0, 10, 320, ILI9341_GREEN);
      //    else tft.fillRect(0, 0, 10, 320, ILI9341_BLACK);
          //if (pushButtonState) tft.fillRect(100, 120, 40, 40, ILI9341_WHITE);
    //else tft.fillRect(100, 120, 40, 40, ILI9341_BLACK);
    
    char dispVal[14];
    
    
    tft.fillRect(90, 0, 20, 30, ILI9341_BLACK);
    tft.setCursor(90,0);
    tft.print(mode);

    tft.fillRect(198, 0, 42, 30, ILI9341_BLACK);
    tft.setCursor(198,0);
    sprintf(dispVal,"%2i",int(gps.satellites.value()));
    tft.print(dispVal);
    
    tft.fillRect(180, 30, 60, 30, ILI9341_BLACK);
    tft.setCursor(180,30);
    sprintf(dispVal,"%3i",int(compassHeading));
    tft.print(dispVal); 
    
    tft.fillRect(180, 60, 60, 30, ILI9341_BLACK);
    tft.setCursor(180,60);
    sprintf(dispVal,"%3i",int(yawAngle));
    tft.print(dispVal);
    
    tft.fillRect(180, 90, 60, 30, ILI9341_BLACK);
    tft.setCursor(180,90);
    sprintf(dispVal,"%3i",int(gps.course.deg()));
    tft.print(dispVal);
    
    tft.fillRect(180, 120, 60, 30, ILI9341_BLACK);
    tft.setCursor(180,120);
    sprintf(dispVal,"%3i",int(goalAngle));
    tft.print(dispVal);
     
    tft.fillRect(162, 150, 78, 30, ILI9341_BLACK);
    tft.setCursor(162,150);
    sprintf(dispVal,"%4.1f",gps.speed.mph());
    tft.print(dispVal);
     
    tft.fillRect(180, 180, 60, 30, ILI9341_BLACK);
    tft.setCursor(180,180);
    sprintf(dispVal,"%3.1f",goalSpeed);
    tft.print(dispVal);

     
    tft.fillRect(180, 210, 60, 30, ILI9341_BLACK);
    tft.setCursor(180,210);
    sprintf(dispVal,"%3i",int(leftMotor));
    tft.print(dispVal);

    tft.fillRect(180, 240, 60, 30, ILI9341_BLACK);
    tft.setCursor(180,240);
    sprintf(dispVal,"%3i",int(rightMotor));
    tft.print(dispVal);

    tft.fillRect(144, 270, 96, 30, ILI9341_BLACK);
    tft.setCursor(144,270);
    sprintf(dispVal,"%5.1f",yawRate);
    tft.print(dispVal);
  }
}


void debugDataHeader(){
  Serial.print("speedK\t");
  Serial.println(speedK);
  Serial.print("speedI\t");
  Serial.println(speedI);
  Serial.print("speedD\t");
  Serial.println(speedD);
  
  Serial.print("turnK\t");
  Serial.println(turnK);
  Serial.print("turnI\t");
  Serial.println(turnI);
  Serial.print("turnD\t");
  Serial.println(turnD);
  
  Serial.print("angleK\t");
  Serial.println(angleK);
  Serial.print("angleI\t");
  Serial.println(angleI);
  Serial.print("angleD\t");
  Serial.println(angleD);
  
    Serial.print("mode");
    Serial.print("\t");
    Serial.print("millis");
    Serial.print("\t");
    Serial.print("rightMotor");
    Serial.print("\t");
    Serial.print("leftMotor");
    Serial.print("\t");
    Serial.print("angleSetting");
    Serial.print("\t");
    Serial.print("speedSetting");
    Serial.print("\t");
    Serial.print("goalSpeed");
    Serial.print("\t");
    Serial.print("gps.speed.mph()");
    Serial.print("\t");
    Serial.print("goalAngle");
    Serial.print("\t");
    Serial.print("gps.course.deg()");
    Serial.print("\t");
    Serial.print("compassHeading");
    Serial.print("\t");
    Serial.print("yawAngle");
    Serial.print("\t");
    Serial.print("yawRate");
    Serial.print("\t");
    Serial.print("sats");
    Serial.print("\t");
    Serial.print("gps.altitude.feet()");
    Serial.print("\t");
    Serial.print("gps.location.lat");
    Serial.print("\t");
    Serial.print("gps.location.lng"); 
    Serial.print("\t");
    Serial.print("fixPointLat");
    Serial.print("\t");
    Serial.print("fixPointLon");
    Serial.print("\t");
    Serial.print("distanceToFixPoint");
    Serial.print("\t");
    Serial.print("courseToFixPoint");
    Serial.print("\t");
    
    Serial.println("CANTXcount");
}
void debugData(){
  if (debugSerialtimer >= 200){
    debugSerialtimer = 0;
    Serial.print(mode);
    Serial.print("\t");
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(rightMotor);
    Serial.print("\t");
    Serial.print(leftMotor);
    Serial.print("\t");
    Serial.print(angleSetting);
    Serial.print("\t");
    Serial.print(speedSetting);
    Serial.print("\t");
    Serial.print(goalSpeed);
    Serial.print("\t");
    Serial.print(gps.speed.mph());
    Serial.print("\t");
    Serial.print(goalAngle);
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

double getCompassHeading(){
  compass.readHeading();
  double tempHeading = compass.heading/10.0 - compassOffset;
  if (tempHeading >= 360) tempHeading -= 360;
  if (tempHeading < 0) tempHeading += 360;
  return tempHeading;
}

double getYawAngle(){
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  double tempyawAngle = euler.x() - yawOffset;
  while (tempyawAngle >= 360) tempyawAngle -= 360;
  while (tempyawAngle < 0) tempyawAngle += 360;
  return tempyawAngle;
}
 
void loop() {
  
  //measure stuff
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  yawRate = gyro.z();
  compassHeading = getCompassHeading();
  yawAngle = getYawAngle();
   
  while (Serial1.available())
     gps.encode(Serial1.read());
  
  //send stuff
  displayData();
  debugData();
  sendCANmessages();

  
  //get user input
  readCANmessages();
  if (CANaliveTimer > 500) mode = 0;
 
  if (mode != currentMode){
    resetOutputs();
    debugDataHeader();
  
  }
  
  if (mode == 0){
    displayMode0();
    
    
  }
  else if (mode == 1){ // Manual Mode 
    displayMode1();
   
    if (mode1started){
      if (speedSettingTimer > speedSetTime) {
        speedSettingTimer = 0;
        if (upButtonState) goalSpeed+=0.1; //mph
        if (downButtonState) goalSpeed-=0.1;
        goalSpeed = constrain(goalSpeed,0,4);
        speedSetting = 22.0 * goalSpeed; //feed forward
      }
      if (courseSettingTimer > courseSetTime) {
        courseSettingTimer = 0;
        if (leftButtonState) goalAngle-=1;
        if (rightButtonState) goalAngle+=1;
        if (goalAngle > 360) goalAngle -= 360;
        if (goalAngle < 0   ) goalAngle += 360;
      }
    }
    else
    {
      speedSetting = 0;
      goalAngle = yawAngle;
      memset(differenceList,0,memorySize) ;
      memset(differenceSpeedList,0,memorySize) ;
    }
    if (upButtonState && pushButtonState) {
      mode1started = true; 
      speedSetting=0; 
      resetYawOffset(); 
      goalAngle = yawAngle; 
      memset(differenceList,0,memorySize) ;
      
    }
    if (downButtonState && pushButtonState){ 
      memset(differenceSpeedList,0,memorySize);
      speedSetting=0; 
      goalSpeed=0; }

  }
  else if (mode == 2){ // turn 90 degrees
    displayMode2();
    if (upButtonState && pushButtonState) mode2started = true;
    
    distanceToFixPoint = gps.distanceBetween(gps.location.lat(), gps.location.lng(), fixPointLat, fixPointLon);
    courseToFixPoint = gps.courseTo(gps.location.lat(), gps.location.lng(), fixPointLat, fixPointLon);
 
  }
  else if (mode == 3){ // anchor
    if (upButtonState && pushButtonState) mode3started = true;
    displayMode3();
    
  }
  else if (mode == 4){ // figure 8
    if (upButtonState && pushButtonState) mode4started = true;
    displayMode4();
    
  }
  else if (mode == 5){ //tune
    if (upButtonState && pushButtonState) mode5started = true;
    displayMode5();
    if (mode5started) {
      if (upButtonState) speedSetting = 150;
      else if (downButtonState) speedSetting = -150;
      else speedSetting = 0;
      if (leftButtonState) turnSetting = 150;
      else if (rightButtonState) turnSetting = -150;
      else turnSetting = 0;
    }
    else
    {
      speedSetting = 0;
      goalAngle = yawAngle; 
    }
    
  }
  else{
    mode = 0;
    speedSetting = 0;
      
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
  delay(50);
  displayTemplate();
  currentMode = mode;
}

void displayMode0(){
  if (mode0displaytimer >= 80){
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
  if (mode1displaytimer >= 80){
    mode1displaytimer = 0;
    
    sprintf(message,"%i Manual",mode);
    txmsg.id=0x211; //sent to the lower right
    txmsg.len=8;
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message," Spd:%3.1f",(goalSpeed));
    txmsg.id=0x212; //sent to the lower right
    for (int j = 0;j<txmsg.len;j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    if (mode1started){
      sprintf(message,"G:%3i Y:",int(goalAngle));
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
  if (mode2displaytimer >= 80){
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
  if (mode3displaytimer >= 80){
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
  if (mode4displaytimer >= 80){
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
  if (mode5displaytimer >= 80){
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

void  calculateMotorOutput(){
  if (!mode == 0){
  

    difference = goalAngle - yawAngle;
    if (difference <= -180)  difference += 360;
    if (difference >= 180)  difference -= 360;

    differenceList[diffIndex] = difference;
    diffIndex+=1;
    if (diffIndex >= memorySize) diffIndex = 0;

    double sum = 0;
    for (int j = 0; j < memorySize; j++){ sum += differenceList[j]; }

    if (gps.satellites.value() > 4 && mode !=5)
    {
      speedDifference = goalSpeed - gps.speed.mph();
      
      differenceSpeedList[diffIndex] = speedDifference;
      diffSpeedIndex+=1;
      if (diffSpeedIndex >= memorySize) diffSpeedIndex = 0;
      double speedSum = 0;
      for (int j = 0; j < memorySize; j++) speedSum += differenceSpeedList[j];
      
      speedSetting += int(speedK*speedDifference + speedI*speedSum);
      
    }

    if (mode!=5) angleSetting = int(angleK*difference + angleI*sum + turnD*yawRate);
  
    
    int tempRightMotor = speedSetting - turnSetting - angleSetting + 92;
    int tempLeftMotor  = speedSetting + turnSetting + angleSetting + 92;
    rightMotor = constrain(tempRightMotor,8,205);
    leftMotor  = constrain(tempLeftMotor,8,205);
  }   
  else
  {
    rightMotor = 92;
    leftMotor  = 92;  
  }
  rightServo.write(rightMotor);
  leftServo.write(leftMotor); 
}

