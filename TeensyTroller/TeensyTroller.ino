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

//PID Gain Constants. Tune these for best results.
double angleK = 4.8;
double angleI = .004;
double angleD = 4;

double speedK = .1;
double speedI = .01;
double speedD = 0;



const int memorySize = 2000;
int32_t differenceList[2000];
int32_t differenceSpeedList[2000];

const uint32_t deltaTms = 50; //milliseconds for calculations and output
double deltaT = 0;
double yawOffset = 0;
double compassOffset = 0;
double initialYaw;

double turnRate = 0.01;
double CANheading;
double headerValue;
double headingReading;
double totalTurn = 0;
double lastAngle = 0;

double omega = 0;
double motorInput = 0;

double gpsSpeed = 0;
double gpsAngle = 0;
byte gpsSats  = 0;
byte gpsFix   = 0;
int dist;
double distK = 0.2;

double waypoints[100][2];
int waypointIndex;
const int stopMotorValue = 92;
const int maxRevMotorValue = 8;
const int maxFwdMotorValue = 208;

elapsedMillis calculateMotorOutputTimer; // this is interrupt based
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
elapsedMillis mode7displaytimer;
elapsedMillis CANaliveTimer;
elapsedMillis speedSettingTimer;
elapsedMillis broadcastCANmodeTimer;
elapsedMillis courseSettingTimer;
elapsedMillis compassReadingTimer;
elapsedMillis gyroReadingTimer;
elapsedMillis delayTimer;
elapsedMillis sineSweepTimer;

const int speedSetTime = 150; //set how quickly the speed changes.
const int courseSetTime = 150; //set how quickly the speed changes.

boolean mode1started = false;
boolean mode2started = false;
boolean mode3started = false;
boolean mode4started = false;
boolean mode5started = false;
boolean mode6started = false;
boolean mode7started = false;

byte mode = 0;
byte currentMode = 0;
byte numberOfModes = 8; //This limits the number of displayed modes.
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
double integral = 0;

int diffIndex = 0;
int diffSpeedIndex = 0;

double difference = 0;
double speedDifference = 0;
double goalAngle = 0;
double goalSpeed = 0;
double turnSetting = 0;
double tempHeading = 0;

int speedSetting;
int angleSetting;

int leftMotor = 92;
int rightMotor = 92;


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
static CAN_message_t txmsg, rxmsg;
uint32_t CANTXcount = 0;
uint32_t CANRXcount = 0;
uint32_t ID = 0;
char message[17] = "                "; //initialize with spaces

// setup the IMU sensor
Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> euler;
imu::Vector<3> gyro;

float yawAngle = 0.0;
float yawRate = 0.0;
float compassHeading = 0.0;

void setup() {

  Serial.begin(115200); //debug console
  delay(500);

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

  bno.setMode(bno.OPERATION_MODE_CONFIG);

  Wire.beginTransmission(BNO055_ADDRESS_A);
  Wire.write(uint8_t(0x07)); //Page ID
  Wire.write(uint8_t(0x01)); // Set to page 1
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(BNO055_ADDRESS_A);
  Wire.write(uint8_t(0x0A)); //Gyro config
  Wire.write(uint8_t(0b00100100)); // Set gyroscope to 125deg/s at 23Hz (see pg 28 of datasheet)
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(BNO055_ADDRESS_A);
  Wire.write(uint8_t(0x07)); //Page ID
  Wire.write(uint8_t(0x00)); // Set to page 1
  Wire.endTransmission();
  delay(10);

  bno.setMode(bno.OPERATION_MODE_AMG);//

  tft.print("Starting Srvo");
  rightServo.attach(23);  // attaches the servo on pin 23 to the servo object
  leftServo.attach(16);  // attaches the servo on pin 16 to the servo object


  tft.println("Starting GPS");
  Serial1.begin(9600);

  tft.print("Starting Comp");
  compass.init();
  byte bSN_LSB = compass.readEEPROM(SN_LSB);
  byte bSN_MSB = compass.readEEPROM(SN_MSB);
  tft.print("S/N:");
  tft.println(word(bSN_MSB, bSN_LSB));
  compass.writeEEPROM(0x14, 0x04); //Set filter to 4.
  compass.exitStandby();

  tft.println("Starting CAN");
  delay(100);
  CANbus.begin();



  strncpy(message, "Fishing ", 8);
  txmsg.id = 0x211; //Send to the upper left
  txmsg.len = 8;
  for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
  CANbus.write(txmsg);
  delay(50);

  strncpy(message, "is great", 8);
  txmsg.id = 0x212; //sent to the upper right
  txmsg.len = 8;
  for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
  CANbus.write(txmsg);
  delay(50);

  strncpy(message, "today. H", 8);
  txmsg.id = 0x221; //sent to the lower left
  txmsg.len = 8;
  for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
  CANbus.write(txmsg);
  delay(50);

  strncpy(message, "ave fun.", 8);
  txmsg.id = 0x222; //sent to the lower right
  txmsg.len = 8;
  for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
  CANbus.write(txmsg);
  delay(50);


  tft.print("CAN msgs sent");

  compass.readHeading();
  yawOffset = compass.heading / 10.0 - compassOffset;

  delay(1000);

  deltaT = double(deltaTms) / 1000.0;

  displayTemplate();
}

void resetCompassOffset() {
  if (gps.speed.mph() > 1.0 && abs(yawRate) < 1.0 ) //compass heading can be realigned to velocity vector
  {
    if (gps.course.deg() >= 270 && compass.heading / 10.0 <= 90)
    {
     // compassOffset = gps.course.deg() - compass.heading / 10.0 + 360;
      compassOffset = gps.course.deg() - CANheading + 360;
    }
    else if (gps.course.deg() <= 90 && compass.heading / 10.0 >= 270)
    {
      compassOffset = gps.course.deg() - compass.heading / 10.0 - 360;
      compassOffset = gps.course.deg() - CANheading - 360;
    }
    else
    {
      //compassOffset = gps.course.deg() - compass.heading / 10.0;
      compassOffset = gps.course.deg() - CANheading;
    }

  }
  else
  {
    compassOffset = 0; // this is the constant angle of misalignment of the sensor and the boat.
  }
}

void resetYawOffset() {
  compassHeading = getCompassHeading();

  if (compassHeading >= 270 && euler.x() <= 90)
  {
    yawOffset = euler.x() - compassHeading -  + 360;
  }
  else if (compassHeading <= 90 && euler.x() >= 270)
  {
    yawOffset = euler.x() - compassHeading - 360;
  }
  else
  {
    yawOffset = euler.x() - compassHeading;
  }
}


void make90rightWaypoints() {
  double x0 = gps.location.lng();
  double y0 = gps.location.lat();
  double radius = 50; //meters
  for (int i = 0; i < 100; i++) {
    double newX = 0;
    double newY = 0;

    //    waypoints[i][2]={x0 + newX,y0 +newY};
  }
}

void sendCANmessages() {
  if (broadcastCANtimer >= 1000) {
    broadcastCANtimer = 0;

    //GPS Messages
    if (gps.location.isUpdated() ) {
      txmsg.id = 0x43e;
      txmsg.len = 8;

      txmsg.buf[0] = byte( (gps.speed.value() & 0xFF000000) >> 24);
      txmsg.buf[1] = byte( (gps.speed.value() & 0x00FF0000) >> 16);
      txmsg.buf[2] = byte( (gps.speed.value() & 0x0000FF00) >>  8);
      txmsg.buf[3] = byte( (gps.speed.value() & 0x000000FF) >>  0);
      txmsg.buf[4] = byte( (gps.course.value() & 0xFF000000) >> 24);
      txmsg.buf[5] = byte( (gps.course.value() & 0x00FF0000) >> 16);
      txmsg.buf[6] = byte( (gps.course.value() & 0x0000FF00) >>  8);
      txmsg.buf[7] = byte( (gps.course.value() & 0x000000FF) >>  0);

      CANbus.write(txmsg);
      CANTXcount++;
    }
  }

  if (broadcastCANmodeTimer >= 100) {
    broadcastCANmodeTimer = 0;
    txmsg.id = 0x210;
    txmsg.len = 8;

    txmsg.buf[0] = numberOfModes;
    txmsg.buf[1] = mode;
    txmsg.buf[2] = 0xFF;
    txmsg.buf[3] = 0xFF;
    txmsg.buf[4] = 0xFF;
    txmsg.buf[5] = 0xFF;
    txmsg.buf[6] = 0xFF;
    txmsg.buf[7] = 0xFF;

    CANbus.write(txmsg);
    CANTXcount++;
  }
}



void readCANmessages() {
  while ( CANbus.read(rxmsg) ) {
    waitingForCANtimer = 0; //reset the can message timeout
    CANRXcount++;
    ID = rxmsg.id;
    if (ID == 0x700) {
      CANaliveTimer = 0;
      mode = rxmsg.buf[0];
      upButtonState = bitRead(rxmsg.buf[1], 0);
      downButtonState = bitRead(rxmsg.buf[1], 1);
      leftButtonState = bitRead(rxmsg.buf[1], 2);
      rightButtonState = bitRead(rxmsg.buf[1], 3);
      pushButtonState = bitRead(rxmsg.buf[1], 4);
    }
    if (ID == 0x43c) {
      CANheading = (rxmsg.buf[0] * 256 + rxmsg.buf[1]) / 10.;
    }
    else if (ID == 0x43d) {
      CANheading = (rxmsg.buf[6] * 256 + rxmsg.buf[7]) / 10.;
    }
    else if (ID == 0x43e) {
      headingReading = (rxmsg.buf[0] * 256 + rxmsg.buf[1]) / 10.;
      gpsSpeed = (rxmsg.buf[2] * 256 + rxmsg.buf[3]) * 1.15078 + 0.5;
      gpsAngle = (rxmsg.buf[4] * 256 + rxmsg.buf[5]);
      gpsSats  = rxmsg.buf[6];
      gpsFix   = rxmsg.buf[7];
    }
    else if (ID == 0x441) {
      headerValue = rxmsg.buf[0] * 256 + rxmsg.buf[1];
      dist = rxmsg.buf[2] * 256 + rxmsg.buf[3];
    }
  }
}


void displayTemplate() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.print("Mode:X Sat:XX");

  tft.setCursor(0, 30);
  tft.print("Comp Head:XXX");

  tft.setCursor(0, 60);
  tft.print("Yaw Angle:XXX");

  tft.setCursor(0, 90);
  tft.print("GPS Headn:XXX");

  tft.setCursor(0, 120);
  tft.print("GoalAngle:XXX");

  tft.setCursor(0, 150);
  tft.print("GPSSpeed:XX.X");

  tft.setCursor(0, 180);
  tft.print("GoalSpeed:X.X");

  tft.setCursor(0, 210);
  tft.print("LeftMotor:XXX");

  tft.setCursor(0, 240);
  tft.print("RghtMotor:XXX");

  tft.setCursor(0, 270);
  tft.print("YawRate:XXX.X");


}

void displayData() {
  if (printTFTtimer > 250) {
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
    tft.setCursor(90, 0);
    tft.print(mode);

    tft.fillRect(198, 0, 42, 30, ILI9341_BLACK);
    tft.setCursor(198, 0);
    sprintf(dispVal, "%2i", int(gps.satellites.value()));
    tft.print(dispVal);

    tft.fillRect(180, 30, 60, 30, ILI9341_BLACK);
    tft.setCursor(180, 30);
    sprintf(dispVal, "%3i", int(compassHeading));
    tft.print(dispVal);

    tft.fillRect(180, 60, 60, 30, ILI9341_BLACK);
    tft.setCursor(180, 60);
    sprintf(dispVal, "%3i", int(yawAngle));
    tft.print(dispVal);

    tft.fillRect(180, 90, 60, 30, ILI9341_BLACK);
    tft.setCursor(180, 90);
    sprintf(dispVal, "%3i", int(gps.course.deg()));
    tft.print(dispVal);

    tft.fillRect(180, 120, 60, 30, ILI9341_BLACK);
    tft.setCursor(180, 120);
    sprintf(dispVal, "%3i", int(goalAngle));
    tft.print(dispVal);

    tft.fillRect(162, 150, 78, 30, ILI9341_BLACK);
    tft.setCursor(162, 150);
    sprintf(dispVal, "%4.1f", gps.speed.mph());
    tft.print(dispVal);

    tft.fillRect(180, 180, 60, 30, ILI9341_BLACK);
    tft.setCursor(180, 180);
    sprintf(dispVal, "%3.1f", goalSpeed);
    tft.print(dispVal);


    tft.fillRect(180, 210, 60, 30, ILI9341_BLACK);
    tft.setCursor(180, 210);
    sprintf(dispVal, "%3i", int(leftMotor));
    tft.print(dispVal);

    tft.fillRect(180, 240, 60, 30, ILI9341_BLACK);
    tft.setCursor(180, 240);
    sprintf(dispVal, "%3i", int(rightMotor));
    tft.print(dispVal);

    tft.fillRect(144, 270, 96, 30, ILI9341_BLACK);
    tft.setCursor(144, 270);
    sprintf(dispVal, "%5.1f", yawRate);
    tft.print(dispVal);
  }
}


void debugDataHeader() {
  Serial.print("speedK\t");
  Serial.println(speedK);
  Serial.print("speedI\t");
  Serial.println(speedI);
  Serial.print("speedD\t");
  Serial.println(speedD);

  Serial.print("angleK\t");
  Serial.println(angleK);
  Serial.print("angleI\t");
  Serial.println(angleI);
  Serial.print("angleD\t");
  Serial.println(angleD);
  Serial.print("mode\t");
  Serial.println(mode);


  Serial.print("millis");
  Serial.print("\t");
  Serial.print("goalAngle");
  Serial.print("\t");
  Serial.print("compassHeading");
  Serial.print("\t");
  Serial.print("difference");
  Serial.print("\t");
  Serial.print("integral");
  Serial.print("\t");
  Serial.print("yawRate");
  Serial.print("\t");
  Serial.print("angleSetting");
  Serial.print("\t");
  Serial.print("goalSpeed");
  Serial.print("\t");
  Serial.print("gps.speed.mph()");
  Serial.print("\t");
  Serial.print("speedSetting");
  Serial.print("\t");
  Serial.print("rightMotor");
  Serial.print("\t");
  Serial.print("leftMotor");
  Serial.print("\t");
  Serial.print("gps.course.deg()");
  Serial.print("\t");
  Serial.print("yawAngle");
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
  Serial.println("Frequency");

}
void debugData() {
  if (debugSerialtimer >= deltaTms) {
    debugSerialtimer = 0;
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(goalAngle);
    Serial.print("\t");
    Serial.print(compassHeading);
    Serial.print("\t");
    Serial.print(difference, 3);
    Serial.print("\t");
    Serial.print(integral, 3);
    Serial.print("\t");
    Serial.print(yawRate, 3);
    Serial.print("\t");
    Serial.print(angleSetting);
    Serial.print("\t");
    Serial.print(goalSpeed);
    Serial.print("\t");
    Serial.print(gps.speed.mph());
    Serial.print("\t");
    Serial.print(speedSetting);
    Serial.print("\t");
    Serial.print(rightMotor);
    Serial.print("\t");
    Serial.print(leftMotor);
    Serial.print("\t");
    Serial.print(gps.course.deg());
    Serial.print("\t");
    Serial.print(yawAngle);
    Serial.print("\t");
    Serial.print(gps.satellites.value());
    Serial.print("\t");
    Serial.print(gps.altitude.feet());
    Serial.print("\t");
    Serial.print(gps.location.lat(), 8);
    Serial.print("\t");
    Serial.print(gps.location.lng(), 8);
    Serial.print("\t");
    Serial.print(fixPointLat, 8);
    Serial.print("\t");
    Serial.print(fixPointLon, 8);
    Serial.print("\t");
    Serial.print(distanceToFixPoint);
    Serial.print("\t");
    Serial.print(courseToFixPoint);
    Serial.print("\t");
    Serial.print(omega, 6);
    Serial.print("\t");

    Serial.println(CANTXcount);
  }
}

double getCompassHeading() {
  compass.readHeading();

  //tempHeading = compass.heading/10.0 - compassOffset; //local compass
  tempHeading = CANheading - compassOffset; //CAN compass

  if (tempHeading >= 360) tempHeading -= 360;
  if (tempHeading < 0) tempHeading += 360;
  return tempHeading;
}

double getYawAngle() {
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  double tempyawAngle = euler.x() - yawOffset;
  while (tempyawAngle >= 360) tempyawAngle -= 360;
  while (tempyawAngle < 0) tempyawAngle += 360;
  return tempyawAngle;
}

void getMeasurements() {
  //measure stuff
  if (gyroReadingTimer >= deltaTms) {
    gyroReadingTimer = 0;
    gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    yawRate = gyro.z();
  }

  if (compassReadingTimer >= 200) {
    compassReadingTimer = 0;
    compassHeading = getCompassHeading();
    yawAngle = getYawAngle();
  }

  //get user input
  readCANmessages();
  if (CANaliveTimer > 500) mode = 0;


  while (Serial1.available())
    gps.encode(Serial1.read());
}

void loop() {

  getMeasurements();

  //send stuff
  displayData();
  sendCANmessages();




  if (mode != currentMode) {
    resetOutputs();
    debugDataHeader();
  }

  if (mode == 0) {
    displayMode0();
  }


  //##############################################################################################
  //# Mode 1: Manual
  //##############################################################################################
  else if (mode == 1) { // Manual Mode
    displayMode1();

    if (mode1started) {

      if (speedSettingTimer > speedSetTime) {
        speedSettingTimer = 0;
        if (upButtonState) goalSpeed += 0.1; //mph
        if (downButtonState) goalSpeed -= 0.1;
        goalSpeed = constrain(goalSpeed, 0, 4);
        speedSetting = 22.0 * goalSpeed; //feed forward
      }
      if (courseSettingTimer > courseSetTime) {
        courseSettingTimer = 0;
        if (leftButtonState) goalAngle -= 1;
        if (rightButtonState) goalAngle += 1;
        if (goalAngle > 360) goalAngle -= 360;
        if (goalAngle < 0   ) goalAngle += 360;
      }
      calculateMotorOutput();


    }
    else
    {
      speedSetting = 0;
      goalAngle = compassHeading;
      rightMotor = stopMotorValue;
      leftMotor = stopMotorValue;
    }

    if (upButtonState && pushButtonState) {
      mode1started = true;
      speedSetting = 0;
      resetYawOffset();
      goalAngle = compassHeading;
      memset(differenceList, 0, sizeof(differenceList)) ;

    }
    if (downButtonState && pushButtonState) {
      memset(differenceSpeedList, 0, sizeof(differenceSpeedList));
      speedSetting = 0;
      goalSpeed = 0;
    }

  }
  //##############################################################################################
  //# Mode 2: Turn 90
  //##############################################################################################
  //##############################################################################################

  else if (mode == 2) { // turn 90 degrees
    displayMode2();
    if (upButtonState && pushButtonState) mode2started = true;
    if (mode2started)
    {
      rightMotor = stopMotorValue;
      leftMotor = stopMotorValue;
    }
    else
    {
      rightMotor = stopMotorValue;
      leftMotor = stopMotorValue;
    }
  }
  //##############################################################################################
  //# Mode 3: Anchor
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################

  else if (mode == 3) { // anchor
    if (upButtonState && pushButtonState) {
      mode3started = true;
      fixPointLat = gps.location.lat();
      fixPointLon = gps.location.lng();
      resetCompassOffset();
      resetYawOffset();
    }

    displayMode3();
    distanceToFixPoint = gps.distanceBetween(gps.location.lat(), gps.location.lng(), fixPointLat, fixPointLon);
    courseToFixPoint = gps.courseTo(gps.location.lat(), gps.location.lng(), fixPointLat, fixPointLon);
    if (mode3started) {
      goalAngle = courseToFixPoint;
      if (distanceToFixPoint > 5) {

        goalSpeed = distanceToFixPoint * distK;
        calculateMotorOutput();
      }
      else
      {
        rightMotor = stopMotorValue;
        leftMotor = stopMotorValue;
      }

    }
  }
  //##############################################################################################
  //# Mode 4: Figure 8
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################

  else if (mode == 4) { // figure 8
    if (upButtonState && pushButtonState) mode4started = true;
    displayMode4();
    if (mode5started) {
      //Put figure8 code here
      rightMotor = stopMotorValue;
      leftMotor = stopMotorValue;

    }
    else
    {
      rightMotor = stopMotorValue;
      leftMotor = stopMotorValue;
    }
  }
  //##############################################################################################
  //# Mode 5: FULL
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################

  else if (mode == 5) { //Full
    if (upButtonState && pushButtonState) mode5started = true;
    displayMode5();

    if (mode5started) {
      debugData();
      if (upButtonState) {
        rightMotor = maxFwdMotorValue;
        leftMotor = maxFwdMotorValue;
      }
      else if (downButtonState) {
        rightMotor = maxRevMotorValue;
        leftMotor = maxRevMotorValue;
      }
      else if (rightButtonState) {
        rightMotor = maxRevMotorValue;
        leftMotor = maxFwdMotorValue;
      }
      else if (leftButtonState) {
        rightMotor = maxFwdMotorValue;
        leftMotor = maxRevMotorValue;
      }
      else
      {
        rightMotor = stopMotorValue;
        leftMotor = stopMotorValue;
      }

    }
  }




  //##############################################################################################
  //# Mode 6: Calibrate Compass
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################

  else if (mode == 6) { //Calibrate
    if (upButtonState && pushButtonState) {
      mode6started = true;
      compass.enterCalMode();
      delay(30);

      totalTurn = 0;
      lastAngle = compassHeading;
      yawOffset = euler.x();
      yawAngle = getYawAngle();

      rightMotor = maxRevMotorValue;
      leftMotor = maxFwdMotorValue;

      rightServo.write(rightMotor);
      leftServo.write(leftMotor);
      delayTimer = 0;
      while (delayTimer < 1000) {
        while (Serial1.available()) gps.encode(Serial1.read());
        sendCANmessages();
      }

    }

    if (downButtonState && pushButtonState) {
      compass.writeEEPROM(0x0A, 0x00); //clear deviation
      delay(10);
      compass.writeEEPROM(0x0B, 0x00);
      delay(10);
      compass.reset();

      delayTimer = 0;
      while (delayTimer < 500) {
        while (Serial1.available()) gps.encode(Serial1.read());
        sendCANmessages();
      }
    }


    displayMode6();

    if (mode6started) {
      debugData();
      double deltaAngle = compassHeading - lastAngle;
      lastAngle = compassHeading;
      if (deltaAngle > 180) totalTurn += 360 - deltaAngle ;
      else if (deltaAngle < -180) totalTurn += deltaAngle + 360;
      else totalTurn += deltaAngle;

      if (abs(totalTurn) < 360)
      {
        rightMotor = maxRevMotorValue;
        leftMotor = maxFwdMotorValue;
      }
      else
      {
        rightMotor = stopMotorValue;
        leftMotor = stopMotorValue;

        rightServo.write(rightMotor);
        leftServo.write(leftMotor);

        strncpy(message, "Turn OK ", 8);
        txmsg.id = 0x212; //sent to the lower right
        for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
        CANbus.write(txmsg);
        CANTXcount++;
        delayTimer = 0;
        while (delayTimer < 1000) {
          while (Serial1.available()) gps.encode(Serial1.read());
          sendCANmessages();
        }
        compass.exitCalMode();
        delay(50);
        compass.exitStandby();
        
        resetCompassOffset();
        rightMotor = maxFwdMotorValue;
        leftMotor = maxFwdMotorValue;

        rightServo.write(rightMotor);
        leftServo.write(leftMotor);

        delayTimer = 0;
        while (delayTimer < 10000) {
          while (Serial1.available()) gps.encode(Serial1.read());
          sendCANmessages();
        }


        strncpy(message, "Finished", 8);
        txmsg.id = 0x212; //sent to the lower right
        for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
        CANbus.write(txmsg);
        CANTXcount++;
        compass.writeEEPROM(0x0A, 0x00); //clear deviation
        delay(10);
        compass.writeEEPROM(0x0B, 0x00);
        delay(10);

        compass.reset();

        delayTimer = 0;
        while (delayTimer < 500) {
          while (Serial1.available()) gps.encode(Serial1.read());
          sendCANmessages();
        }

        compass.readHeading();

        int  deviation = gps.course.deg() * 10 - compass.heading; // Not sure if this is the correct deviation method.

        compass.writeEEPROM(0x0A, lowByte(deviation)); //LSB of deviation
        compass.writeEEPROM(0x0B, highByte(deviation)); //MSB of deviation
        delay(50);

        compass.reset();
        delayTimer = 0;
        while (delayTimer < 500) {
          while (Serial1.available()) gps.encode(Serial1.read());
          sendCANmessages();
        }
        rightMotor = stopMotorValue;
        leftMotor = stopMotorValue;


        mode6started = false;
      }
    }
    else
    {
      rightMotor = stopMotorValue;
      leftMotor = stopMotorValue;
    }
  }



  //##############################################################################################
  //# Mode 7: Frequency Sweep
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################

  else if (mode == 7) { //Calibrate
    if (upButtonState && pushButtonState) {
      mode7started = true;
      sineSweepTimer = 0;

    }

    displayMode7();

    if (mode7started) {

      while (sineSweepTimer <= 10000) //number of milliseconds
      {
        getMeasurements();
        sendCANmessages();
        debugData();
        displayData();
        displayMode7();

        rightMotor = stopMotorValue + 50; //go straight for 10 seconds
        leftMotor = stopMotorValue + 50;

        rightServo.write(rightMotor);
        leftServo.write(leftMotor);

        if (downButtonState && pushButtonState) {//abort
          mode7started = false;
          resetOutputs();
          break;
        }
      }

      sineSweepTimer = 0;

      while (sineSweepTimer <= 250000) //number of milliseconds
      {
        getMeasurements();
        sendCANmessages();
        debugData();
        displayData();
        displayMode7();

        omega =  sineSweepTimer / 200000.0;
        motorInput = 50 * sin( 2 * omega * PI * sineSweepTimer / 1000.);
        rightMotor = stopMotorValue + 50 + int(motorInput);
        leftMotor = stopMotorValue + 50 - int(motorInput);

        rightServo.write(rightMotor);
        leftServo.write(leftMotor);

        if (downButtonState && pushButtonState) { //abort
          mode7started = false;
          resetOutputs();
          break;
        }
      }
      mode7started = false;
    }
    else
    {
      rightMotor = stopMotorValue;
      leftMotor = stopMotorValue;
    }

  }



  //////////////////////////////////////////////////Default
  else
  {
    rightMotor = stopMotorValue;
    leftMotor = stopMotorValue;
  }

  /////////////////////////////////////////////////////
  //always send the updates to the servos
  rightServo.write(rightMotor);
  leftServo.write(leftMotor);

}
///////////////////////////////////////////////
void resetOutputs() {
  speedSetting = 0;
  mode1started = false;
  mode2started = false;
  mode3started = false;
  mode4started = false;
  mode5started = false;
  mode6started = false;
  mode7started = false;

  compass.exitStandby();

  tft.fillScreen(ILI9341_GREEN);
  delay(50);
  displayTemplate();
  currentMode = mode;

  //reset the integrator
  memset(differenceList, 0, sizeof(differenceList)) ;
  memset(differenceSpeedList, 0, sizeof(differenceSpeedList)) ;

  rightMotor = stopMotorValue;
  leftMotor  = stopMotorValue;

}

void displayMode0() {
  if (mode0displaytimer >= 80) {
    mode0displaytimer = 0;

    sprintf(message, "%i  OFF  ", mode);
    txmsg.id = 0x211; //sent to the lower right
    txmsg.len = 8;
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message, " Sats:%2i ", int(gps.satellites.value()));
    txmsg.id = 0x212; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message, "H:%3i C:", int(gps.course.deg()));
    txmsg.id = 0x221; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message, "%3i S:%2i", int(compassHeading), int(gps.speed.mph()) );
    txmsg.id = 0x222; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;
  }
}


void displayMode1() {
  if (mode1displaytimer >= 80) {
    mode1displaytimer = 0;

    sprintf(message, "%i Manual", mode);
    txmsg.id = 0x211; //sent to the lower right
    txmsg.len = 8;
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message, " Spd:%3.1f", (goalSpeed));
    txmsg.id = 0x212; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    if (mode1started) {
      sprintf(message, "G:%3i C:", int(goalAngle));
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      sprintf(message, "%3i S:%2i", int(compassHeading), int(gps.speed.mph()) );
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message, "Butn+Up ", 8);
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      strncpy(message, "to Start", 8);
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }
  }
}


void displayMode2() {
  if (mode2displaytimer >= 80) {
    mode2displaytimer = 0;

    sprintf(message, "%i Turn90", mode);
    txmsg.id = 0x211; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message, " Sats:%2i  ", int(gps.satellites.value()));
    txmsg.id = 0x212; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    if (mode2started) {
      sprintf(message, "H:%3i C:", int(gps.course.deg()));
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      sprintf(message, "%3i S:%2i", int(compassHeading), int(gps.speed.mph()) );
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message, "Butn+Up ", 8);
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      strncpy(message, "to Start", 8);
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }


  }
}


void displayMode3() {
  if (mode3displaytimer >= 80) {
    mode3displaytimer = 0;

    sprintf(message, "%i Anchor", mode);
    txmsg.id = 0x211; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message, " Crs:%3i", int(courseToFixPoint));
    txmsg.id = 0x212; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    if (mode3started) {
      sprintf(message, "D:%3i C:", int(distanceToFixPoint));
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      sprintf(message, "%3i S%3.1f", int(compassHeading), goalSpeed);
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message, "Butn+Up ", 8);
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      strncpy(message, "to Start", 8);
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }


  }
}


void displayMode4() {
  if (mode4displaytimer >= 80) {
    mode4displaytimer = 0;

    mode1displaytimer = 0;
    sprintf(message, "%i Fig. 8", mode);
    txmsg.id = 0x211; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message, " Sats:%2i", int(gps.satellites.value()));
    txmsg.id = 0x212; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    if (mode4started) {
      sprintf(message, "H:%3i C:", int(gps.course.deg()));
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      sprintf(message, "%3i S:%2i", int(compassHeading), int(gps.speed.mph()) );
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message, "Butn+Up ", 8);
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      strncpy(message, "to Start", 8);
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }


  }
}


void displayMode5() {
  if (mode5displaytimer >= 80) {
    mode5displaytimer = 0;

    mode1displaytimer = 0;
    sprintf(message, "%i Full  ", mode);
    txmsg.id = 0x211; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message, " Sats:%2i  ", int(gps.satellites.value()));
    txmsg.id = 0x212; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    if (mode5started) {
      sprintf(message, "H:%3i C:", int(gps.course.deg()));
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      sprintf(message, "%3i S:%2i", int(compassHeading), int(gps.speed.mph()) );
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message, "Butn+Up ", 8);
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      strncpy(message, "to Start", 8);
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }

  }
}


void displayMode6() {
  if (mode6displaytimer >= 80) {
    mode6displaytimer = 0;

    sprintf(message, "%i Calib ", mode);
    txmsg.id = 0x211; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message, "Comp %3i", int(compassHeading));
    txmsg.id = 0x212; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    if (mode6started) {
      sprintf(message, "Turn:%4i H:%4i", int(totalTurn), int(gps.course.deg()));
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j + 8];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message, "Butn+Up ", 8);
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      strncpy(message, "to Start", 8);
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }

  }
}

void displayMode7() {
  if (mode7displaytimer >= 80) {
    mode7displaytimer = 0;

    sprintf(message, "%i Freq. ", mode);
    txmsg.id = 0x211; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    sprintf(message, "Sweep %2i", int(motorInput));
    txmsg.id = 0x212; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    CANbus.write(txmsg);
    CANTXcount++;

    if (mode7started) {
      sprintf(message, "%5.4f H:%3i S:%2.1f", omega, int(gps.course.deg()), gps.speed.mph());
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j + 8];
      CANbus.write(txmsg);
      CANTXcount++;
    }
    else
    {
      strncpy(message, "Butn+Up ", 8);
      txmsg.id = 0x221; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;

      strncpy(message, "to Start", 8);
      txmsg.id = 0x222; //sent to the lower right
      for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
      CANbus.write(txmsg);
      CANTXcount++;
    }

  }
}

void  calculateMotorOutput() {
  if (calculateMotorOutputTimer >= deltaTms) {
    calculateMotorOutputTimer = 0;



    // difference = goalAngle - yawAngle; // using BNO055 as input

    difference = goalAngle - compassHeading; //using compass for input
    if (difference <= -180)  difference += 360;
    if (difference >= 180)  difference -= 360;

    differenceList[diffIndex] = int32_t(difference * 1000);
    diffIndex += 1;
    if (diffIndex >= memorySize) diffIndex = 0;

    int32_t sum = 0;
    for (int j = 0; j < memorySize; j++) {
      sum += differenceList[j];
    }

    integral = double(sum) / 1000.0 * deltaT;

    if (gps.satellites.value() > 4)
    {
      speedDifference = goalSpeed - gps.speed.mph();

      differenceSpeedList[diffIndex] = int32_t(speedDifference * 1000);
      diffSpeedIndex += 1;
      if (diffSpeedIndex >= memorySize) diffSpeedIndex = 0;

      int32_t speedSum = 0;
      for (int j = 0; j < memorySize; j++) speedSum += differenceSpeedList[j];

      double integralSpeed = double(speedSum) / 1000.0 * deltaT;

      speedSetting += int(speedK * speedDifference + speedI * integralSpeed);

    }

    angleSetting = int(angleK * difference + angleI * integral + angleD * yawRate);


    int tempRightMotor = speedSetting - turnSetting - angleSetting + stopMotorValue;
    int tempLeftMotor  = speedSetting + turnSetting + angleSetting + stopMotorValue;
    rightMotor = constrain(tempRightMotor, maxFwdMotorValue, maxRevMotorValue);
    leftMotor  = constrain(tempLeftMotor, maxFwdMotorValue , maxRevMotorValue);


    //print
    debugData();
  }

}

