/*
   The Teensy Troller
   A program to implement an autopilot with two trolling motors on a ski boat.

   By Jeremy Daily

   Released under the beer license: If you like it, I'd like to share a beer with you if we ever meet.
*/
#include <TinyGPS++.h> // Used to read the data from the GPS.
#include <FlexCAN.h> // The CAN library for the Teensy used to communicate with the Joystick
#include "SPI.h" //Serial Peripherial Interface
#include "ILI9341_t3.h" // The library for the TFT display for the Teensy 3.2
#include  <i2c_t3.h> // the I2C library that replaces Wire.h for the Teensy 3.2
#include <SFE_HMC6343.h> //Sparkfun's library for the digital compass
#include <Servo.h> // Used to send pulses to the motor controller
#include <EEPROM.h> //used to store compass declination angles and calibration results

float turnRate = 1.500; //degrees per second


#define compassOffsetAddress 0
#define CANcompassOffsetAddress 8
double compassOffset = -10.8;// True - measured, so measured + offset = true


//PID Gain Constants. Tune these for best results.
//Field Test result worked with angleK = 4, angleI = .3, angleD = 25. Determined on 20 July 2016 on Skiatook Lake
double angleK = 8;
double angleI = .3;
double angleD = 30;

//These still need tuned, but they seem to work.
double speedK = .1;
double speedI = .3;
double speedD = 0;


//Set up the Kalman filter to smooth the data from the Compass and Rate Gyro.
// These must be defined before including TinyEKF.h
#define N 3     // three state values: yawAngle, yawRate, speed
#define M 4     // Four measurements: compassHeading, rateGyro, GPS speed, CAN Speed

//Call the Extended Kalman filter to set up sensor fusion for the rate gyro and the compass.
#include <TinyEKF.h>

//declare this here so it can be used in the Fuser model.
double deltaT = 0.05;

class Fuser : public TinyEKF {

  public:

    Fuser()
    {
      // We approximate the process noise using a small constant
      this->setQ(0, 0, .0001);
      this->setQ(1, 1, .0001);
      this->setQ(2, 2, .0001);
      this->setQ(3, 3, .0001);

      // Same for measurement noise (MxM diagnonal matrix assumes no covariance)
      this->setR(0, 0, 1.5); //Varianance of the noise from the Compass (deg)^2
      this->setR(1, 1, .002); //Variance of the noise from the rate gyro (deg/s)^2
      this->setR(2, 2, .05); //Noise from the GPS speed
      this->setR(3, 3, .05); //Noise from the second GPS

    }

  protected:

    void model(double fx[N], double F[N][N], double hx[M], double H[M][N])
    {
      // Process model is f(x) = x
      fx[0] = this->x[0] - this->x[1] * deltaT; //degrees
      fx[1] = this->x[1]; // deg/s
      fx[2] = this->x[2]; // from 1 GPS
      //fx[3] = this->x[2]; // from the other GPS


      //Process model Jacobian
      F[0][0] = 1; //df[0]/dx[0]
      F[0][1] = deltaT; //df[0]/dx[1]
      F[1][0] = 0; //df[1]/dx[0]
      F[1][1] = 1; //df[1]/dx[1]

      F[2][2] = 1;
      F[2][3] = 0;//deltaT*21.9545;
      F[3][2] = 0;
      F[3][3] = 1;

      // Measurement function
      hx[0] = this->x[0]; // Yaw Angle from previous state
      hx[1] = this->x[1]; // yaw Rate from previous state
      hx[2] = this->x[2]; // Speed from previous state
      hx[3] = this->x[2]; // Speed acceleration from previous state

      // Jacobian of measurement function
      H[0][0] = 1;       // Yaw Angle from previous state
      H[1][1] = 1;       // yaw Rate from previous state
      H[2][2] = 1;
      H[3][3] = 1;

    }
};

Fuser ekf;

double ekfYawAngle;
double ekfYawRate;
double ekfSpeed;
//double ekfAccel;

char topLine[17];
char botLine[17];

double gyroScaleFactor = 0.003814697265625; //Set to 125deg/s / 2^15
double gyroOffset = -0.122513335; //Used to zero out the rate gyro when it is still. Uses a longterm average.

double biasSetting = 1.00; // adjust this value to make the boat go straight in full mode. This the compensation coefficent for the right motor

uint8_t headingCount = 0;
double headingSum = 0;
double CANheadingSum = 0;


const int memorySize = 2400;
int32_t differenceList[2400];
int32_t differenceSpeedList[2400];

double accelX = 0;

const uint32_t deltaTms = 50; //milliseconds for calculations and output
const uint32_t modeDisplayPeriod = 100;

double yawOffset = 0;
double CANcompassOffset = 0;
boolean needsRealigned  = true;
boolean needsToPrint = true;

double CANheading;
double CANcompassHeading;
double headerValue;
double headingReading;
double totalTurn = 0;
double lastAngle = 0;
double targetAngle = 0;
double endAngle = 0;
double startAngle = 0;

double omega = 0;
double motorInput = 0;

double gpsSpeed = 0;
double gpsAngle = 0;
byte gpsSats  = 0;
byte gpsFix   = 0;
int dist;
double distK = 0.2;

const int stopMotorValue = 92;
const int maxRevMotorValue = 8;
const int maxFwdMotorValue = 208;

elapsedMillis upperLeftTimer = 0;
elapsedMillis upperRightTimer = 20;
elapsedMillis lowerLeftTimer = 40;
elapsedMillis lowerRightTimer = 60;

elapsedMillis calculateMotorOutputTimer; // this is interrupt based
elapsedMillis broadcastCANtimer; //set up intervals
elapsedMillis waitingForCANtimer;
elapsedMillis debugSerialtimer;
elapsedMillis modeDisplayTimer;
elapsedMillis CANaliveTimer;
elapsedMillis SerialAliveTimer;
elapsedMillis speedSettingTimer;
elapsedMillis broadcastCANmodeTimer;
elapsedMillis courseSettingTimer;
elapsedMillis gyroReadingTimer;

elapsedMillis fig8timer;
elapsedMillis turnTimer;
elapsedMillis turnUpdateTimer;
elapsedMillis resetCompassTimer;

const int speedSetTime = 150; //set how quickly the speed changes.
const int courseSetTime = 150; //set how quickly the Course inputs changes.

boolean mode1started = false;
boolean mode2started = false;
boolean mode3started = false;
boolean mode4started = false;
boolean mode5started = false;
boolean mode6started = false;
boolean mode7started = false;

byte mode = 0;
byte currentMode = 0;
byte numberOfModes = 5; //This limits the number of displayed modes.
byte fig8phase = 0;

boolean rightButtonState = LOW;
boolean leftButtonState = LOW;
boolean downButtonState = LOW;
boolean pushButtonState = LOW;
boolean upButtonState = LOW;
boolean greenButtonState = LOW;
boolean redButtonState = LOW;


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
double turnGoal = 0;
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
//ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

SFE_HMC6343 compass; // Declare the sensor object

Servo rightServo;  // create servo object to control a servo
Servo leftServo;  // create servo object to control a servo

//Set up CAN messaging
FlexCAN CANbus();
static CAN_message_t txmsg, rxmsg;
uint32_t CANTXcount = 0;
uint32_t CANRXcount = 0;
uint32_t ID = 0;
char message[17] = "                "; //initialize with spaces

int speedAdjust;

double compassHeading = 0.0;

boolean rightTurn = false;
boolean leftTurn = false;

float yawAngleRaw = 0;

#include "BNO055.h"




void setup() {
  delay(200);
  Wire.begin();
  Serial.println("Welcome to the Teensy Troller. We will get setup first.");

  Serial.print("Starting Bosch BNO055 IMU Sensor... ");
  uint8_t gyroConfig = 0;
  while (gyroConfig != 0b00101100) {
    BNOwrite(BNO055_PAGE_ID_ADDR, 0); // Set to page 0
    delay(40);
    BNOwrite(BNO055_SYS_TRIGGER_ADDR, 0b00100000);
    delay(40);
    BNOwrite(BNO055_SYS_TRIGGER_ADDR, 0b10000000);
    delay(750);

    BNOwrite(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG); //Set to configuration Mode
    delay(20);
    BNOwrite(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG); //Set to configuration Mode
    delay(20);
    BNOwrite(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG); //Set to configuration Mode
    delay(20);
    Serial.println("Done.");

    // Set to use external oscillator
    Serial.print("Setting up for External oscillator... ");
    if (BNOread(BNO055_SYS_CLK_STAT_ADDR) == 0) {
      BNOwrite(BNO055_SYS_TRIGGER_ADDR, 0b10000000);
      Serial.println("BNO055 set to use external oscillator");
    }
    else
    {
      BNOwrite(BNO055_SYS_TRIGGER_ADDR, 0b10000000);
      Serial.println("Problem setting BNO055 to use external oscillator.");
    }
    delay(10);

    Serial.print("Setting up Rate Gyro... ");
    BNOwrite(BNO055_PAGE_ID_ADDR, 1); // Set to page 1
    delay(10);
    BNOwrite(GYRO_CONF_0, 0b00101100); // Set gyroscope to 125deg/s at 23Hz (see pg 28 of datasheet)
    delay(10);
    BNOwrite(GYRO_CONF_1, 0); // Set gyroscope  (see pg 28 of datasheet)
    delay(10);


    Serial.println("Done.");
    Serial.print("Setting up Magnetometer... ");
    BNOwrite(MAG_CONF, 0b00011001); //
    delay(10);
    Serial.println("Done.");

    Serial.print("Setting up Accelerometer... ");
    BNOwrite(ACC_CONF, 0); //
    delay(10);
    Serial.println("Done.");

    Serial.print("Setting up Units... ");
    BNOwrite(BNO055_PAGE_ID_ADDR, 0); // Set to page 0
    delay(10);
    BNOwrite(BNO055_UNIT_SEL_ADDR, 0b00010000); // Set units to M/s^2 Deg/sec, Deg, Deg F
    delay(10);
    Serial.println("Done.");

    Serial.print("Turning on Sensors... ");
    BNOwrite(BNO055_OPR_MODE_ADDR, OPERATION_MODE_AMG); //
    //BNOwrite(BNO055_OPR_MODE_ADDR,OPERATION_MODE_NDOF);//
    delay(10);
    BNOwrite(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    delay(10);
    Serial.println("Done.");

    Serial.println("Verifying BNO055 Settings...");
    gyroConfig = getBNO055Status();
  }
  Serial.println("Done.");



  Serial.print("Starting Servos... ");

  rightServo.attach(16);  // attaches the servo on pin 16 to the servo object
  leftServo.attach(23);  // attaches the servo on pin 23 to the servo object
  Serial.println("Done.");

  Serial.print("Starting GPS... ");
  Serial1.begin(9600);
  delay(300);
  Serial1.println("$PMTK251,57600*2C"); //Set Baud Rate to 57600
  delay(100);
  Serial1.flush();
  Serial1.end();
  Serial.println("Setting GPS to 57600 baud... ");
  Serial1.begin(57600);
  Serial1.println("$PMTK251,57600*2C"); //Set Baud Rate to 57600

  Serial.println("Setting GPS to update at 5 Hz... ");
  Serial1.println("$PMTK220,200*2C"); //update at 5 Hz
  delay(100);
  Serial1.println("$PMTK300,200,0,0,0,0*2F"); //position fix update to 5 Hz
  for (int i = 0; i < 100; i++) {
    if (Serial1.available()) Serial.write(Serial1.read());
  }
  Serial.println("\nDone.");


  Serial.println("Starting Compass... ");
  compass.init();
  byte bSN_LSB = compass.readEEPROM(SN_LSB);
  byte bSN_MSB = compass.readEEPROM(SN_MSB);
  Serial.print("Compass Serial Number: ");
  Serial.println(word(bSN_MSB, bSN_LSB));
  compass.writeEEPROM(0x14, 0x00); //Set filter to 0.
  compass.exitStandby();
  delay(100);
  Serial.println("\nDone.");

  Can0.begin(500000);

  strncpy(message, "Fishing ", 8);
  txmsg.id = 0x211; //Send to the upper left
  txmsg.len = 8;
  for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
  Can0.write(txmsg);
  delay(50);

  strncpy(message, "is great", 8);
  txmsg.id = 0x212; //sent to the upper right
  txmsg.len = 8;
  for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
  Can0.write(txmsg);
  delay(50);

  strncpy(message, "today. H", 8);
  txmsg.id = 0x221; //sent to the lower left
  txmsg.len = 8;
  for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
  Can0.write(txmsg);
  delay(50);

  strncpy(message, "ave fun.", 8);
  txmsg.id = 0x222; //sent to the lower right
  txmsg.len = 8;
  for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
  Can0.write(txmsg);
  delay(50);

  //tft.print("CAN msgs sent");

  delay(100);
  EEPROM.get(compassOffsetAddress, compassOffset);
  EEPROM.get(CANcompassOffsetAddress, CANcompassOffset);


  compassHeading = getCompassHeading();
  ekf.setX(0, compassHeading);
  deltaT = double(deltaTms) / 1000.0;



//  displayTemplate(); //tft display
  //  Serial.print("Compass");
  //  Serial.print("\t");
  //  Serial.print("ekfCompass");
  //  Serial.print("\t");
  //  Serial.print("rateGyro");
  //  Serial.print("\t");
  //  Serial.print("ekfRateGyro");
  //  Serial.print("GPSspeed");
  //  Serial.print("\t");
  //  Serial.print("ekfSpeed");
  //  Serial.print("\t");
  //  Serial.print("Accel");
  //  Serial.print("\t");
  //  Serial.println("ekfAccel");

}


void sendCANmessages() {
  if (broadcastCANtimer >= 200) {
    broadcastCANtimer = 0;

    //GPS Messages
    if (gps.speed.isUpdated() ) {
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

      Can0.write(txmsg);
      CANTXcount++;
    }
  }

  if (broadcastCANmodeTimer >= deltaTms) {
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

    Can0.write(txmsg);

  }
}



void readCANmessages() {
  
  uint8_t buttonByte;
  
  if ( Can0.read(rxmsg) && SerialAliveTimer < 500) { //Give precedent to the serial connections
    waitingForCANtimer = 0; //reset the can message timeout
    ID = rxmsg.id;
    if (ID == 0x700) {
      CANaliveTimer = 0;
      mode = rxmsg.buf[0];
      buttonByte = rxmsg.buf[1];
    }
    if (ID == 0x43c) {
      CANheading = (rxmsg.buf[0] * 256 + rxmsg.buf[1]) / 10.;
    }
    else if (ID == 0x43d) {
      CANheading = (rxmsg.buf[6] * 256 + rxmsg.buf[7]) / 10.;
    }
    else if (ID == 0x43e) {
      headingReading = (rxmsg.buf[0] * 256 + rxmsg.buf[1]) / 10.;
      gpsSpeed = (rxmsg.buf[2] * 256 + rxmsg.buf[3]) * 1.15078; // + 0.5;
      gpsAngle = (rxmsg.buf[4] * 256 + rxmsg.buf[5]);
      gpsSats  = rxmsg.buf[6];
      gpsFix   = rxmsg.buf[7];
    }
    else if (ID == 0x441) {
      headerValue = rxmsg.buf[0] * 256 + rxmsg.buf[1];
      dist = rxmsg.buf[2] * 256 + rxmsg.buf[3];
    }
  }

  /*
   * Recieve and process Serial commands. THe structure is:
   * Byte 0: length (0 to 255)
   * Byte 1: command (0 to 255)
   * Bytes 2 to 2+length: command values less than 62 bytes.
   */
  if (Serial.available() >= 2)
  {
    SerialAliveTimer = 0;
    char commandLength = Serial.read();
    char commandPrefix = Serial.read();
    char commandValue[commandLength];
    for (uint8_t i = 0; i<commandLength; i++) commandValue[i] = Serial.read();
    
    if (commandPrefix == 'b')  buttonByte = commandValue[0]; //Button
    else if (commandPrefix == 'm') { //mode
      mode = commandValue[0];
      //sendCANModeCommand(); 
    }
    else if (commandPrefix == 't'){ //turn
      int adjustment = atoi(commandValue);
      goalAngle += adjustment;
    }
    else if (commandPrefix == 'k'){ // set the proportional gain for the angle controller
      angleK = atof(commandValue);
    }
    else if (commandPrefix == 'i'){ // set the integral gain for the angle controller
      angleI = atof(commandValue);
    }
    else if (commandPrefix == 'd'){ // set the derivative gain for the angle controller
      angleD = atof(commandValue);
    }
    else if (commandPrefix == 'K'){ // set the proportional gain for the speed controller
      speedK = atof(commandValue);
    }
    else if (commandPrefix == 'I'){ // set the integral gain for the speed controller
      speedI = atof(commandValue);
    }
    else if (commandPrefix == 'D'){ // set the derivative gain for the speed controller
      speedD = atof(commandValue);
    }
    else if (commandPrefix == 'r'){ // reset the integrator
      memset(differenceList, 0, sizeof(differenceList)) ;
    }
    else if (commandPrefix == 'R'){ // reset the integrator for Speed
      memset(differenceSpeedList, 0, sizeof(differenceSpeedList)) ;
    }

  }
  upButtonState =    bitRead(buttonByte, 0);
  downButtonState =  bitRead(buttonByte, 1);
  leftButtonState =  bitRead(buttonByte, 2);
  rightButtonState = bitRead(buttonByte, 3);
  pushButtonState =  bitRead(buttonByte, 4);
  greenButtonState = bitRead(buttonByte, 6);
  redButtonState =   bitRead(buttonByte, 7);
}

    
void debugDataHeader(){
  if (gps.date.isUpdated())
  {
    Serial.print("GPS Date\t");
    Serial.print(gps.date.year());
    Serial.print("-");
    Serial.print(gps.date.month());
    Serial.print("-");
    Serial.println(gps.date.day());
  }

  if (gps.time.isUpdated())
  {
    Serial.print("GPS Time\t");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
  }

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
  Serial.print("compassOffset\t");
  Serial.println(compassOffset);
  Serial.print("CANcompassOffset\t");
  Serial.println(CANcompassOffset);
  

  Serial.print("Time [ms]\t");
  Serial.print("goalAngle [deg]\t");
  Serial.print("Compass [deg]\t");
  Serial.print("GPS Course [deg]\t");
  Serial.print("CANcompassHeading [deg]\t");
  Serial.print("ekfYawAngle\t");
  Serial.print("integral [deg]\t");
  Serial.print("rateGyro [deg/s]\t");
  Serial.print("ekfYawRate [deg/s]\t");
  Serial.print("angleSetting\t");
  Serial.print("goalSpeed [mph]\t");
  Serial.print("GPSspeed [mph]\t");
  Serial.print("CANGPSspeed [mph]\t");
  Serial.print("ekfSpeed [mph]\t");
  Serial.print("speedAdjust\t");
  Serial.print("speedSetting\t");
  Serial.print("rightMotor\t");
  Serial.print("leftMotor\t");
  Serial.print("gps.location.lat\t");
  Serial.print("gps.location.lng\t");
  Serial.print("distanceToFixPoint [m]\t");
  Serial.print("courseToFixPoint [deg]\t");
  Serial.print("fig8phase\t");
  Serial.print("fig8timer\t");
  Serial.println("turnSetting\t");
  //Serial.println("Frequency");

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
    Serial.print(gps.course.deg());
    Serial.print("\t");
    Serial.print(CANcompassHeading);
    Serial.print("\t");
    Serial.print(ekfYawAngle);
    Serial.print("\t");
    Serial.print(integral, 3);
    Serial.print("\t");
    Serial.print(yawRate, 3);
    Serial.print("\t");
    Serial.print(ekfYawRate, 3);
    Serial.print("\t");
    Serial.print(angleSetting);
    Serial.print("\t");
    Serial.print(goalSpeed);
    Serial.print("\t");
    Serial.print(gps.speed.mph());
    Serial.print("\t");
    Serial.print(gpsSpeed);
    Serial.print("\t");
    Serial.print(ekfSpeed);
    Serial.print("\t");
    Serial.print(speedAdjust);
    Serial.print("\t");
    Serial.print(speedSetting);
    Serial.print("\t");
    Serial.print(rightMotor);
    Serial.print("\t");
    Serial.print(leftMotor);
    Serial.print("\t");
    Serial.print(gps.location.lat(), 8);
    Serial.print("\t");
    Serial.print(gps.location.lng(), 8);
    Serial.print("\t");
    Serial.print(distanceToFixPoint);
    Serial.print("\t");
    Serial.print(courseToFixPoint);
    Serial.print("\t");
    Serial.print(fig8phase);
    Serial.print("\t");
    Serial.print(fig8timer);
    Serial.print("\t");
    Serial.println(turnSetting);
    //Serial.print("\t");
    //Serial.println(omega, 6);
  }
}


void resetCompassOffset() {
  if (needsRealigned) {
    if (ekfSpeed > 15.0 && abs(yawRate) < 1.0) //compass heading can be realigned to velocity vector
    {

      if (resetCompassTimer >= 200)
      {
        resetCompassTimer = 0;
        headingSum += compass.heading / 10.0;
        CANheadingSum += CANheading;
        headingCount += 1;
        Serial.print(headingCount);
        Serial.print("\t");
        Serial.println(headingSum);
      }

      if (headingCount >= 20) {
        needsRealigned = false;
        compassOffset =   gps.course.deg() - headingSum / headingCount;
        CANcompassOffset =  gps.course.deg() - CANheadingSum / headingCount;

        if (CANcompassOffset < -180) CANcompassOffset += 360;
        if (CANcompassOffset > 180) CANcompassOffset -= 360;
        if (compassOffset < -180) compassOffset += 360;
        if (compassOffset > 180) compassOffset -= 360;

        Serial.print("Setting Compass Offsets Based on GPS heading: compassOffset = ");
        Serial.print(compassOffset);
        Serial.print(", CANcompassOffset = ");
        Serial.println(CANcompassOffset);

        EEPROM.put(compassOffsetAddress, compassOffset);
        EEPROM.put(CANcompassOffsetAddress, CANcompassOffset);
        compassHeading = getCompassHeading();
        ekf.setX(0, compassHeading);

      }
    }
  }
}

double getCompassHeading() {
  compass.readHeading();
  tempHeading = compass.heading / 10.0 + compassOffset; //local compass
  CANcompassHeading = CANheading + CANcompassOffset;

  if (ekfYawAngle > 270 && tempHeading < 90) tempHeading += 360;
  if (ekfYawAngle < 90 && tempHeading > 270) tempHeading -= 360;
  
  return tempHeading;
}


void getMeasurements() {
  //measure stuff
  if (gyroReadingTimer >= deltaTms) {
    gyroReadingTimer = 0;

    yawRate = BNOgetYawRate();
    compassHeading = getCompassHeading();

    // accelX = BNOgetAccelX();

    // Send these measurements to the EKF
    double z[M] = {compassHeading, yawRate, gps.speed.mph(), gpsSpeed};
    if (mode != 6) ekf.step(z);

    // Report measured and predicted/fused values
    ekfYawAngle = ekf.getX(0);
    ekfYawRate = ekf.getX(1);
    ekfSpeed = ekf.getX(2);

    if (ekfYawAngle >= 360) ekfYawAngle -= 360;
    if (ekfYawAngle < 0)    ekfYawAngle += 360;
    ekf.setX(0, ekfYawAngle);
  }
  
  while (Serial1.available()) gps.encode(Serial1.read());
}

void loop() {

  getMeasurements();

  //send stuff
  sendCANmessages();
  
  //get user input
  readCANmessages();
  if (CANaliveTimer > 500 && SerialAliveTimer > 500) mode = 0;
  
  if (mode != currentMode) {
    resetOutputs();
    debugDataHeader();
  }

  if (mode == 0) {
    displayDefault();
    resetCompassOffset();
  }

  //##############################################################################################
  //# Mode 1: Manual
  //##############################################################################################
  else if (mode == 1) { // Manual Mode
    displayNormalOperation();

    if (mode1started) {
      if (!rightButtonState && !leftButtonState) turnSetting = 0;
      if (speedSettingTimer > speedSetTime) {
        speedSettingTimer = 0;
        if (upButtonState && !pushButtonState) goalSpeed += 0.1; //mph
        if (downButtonState && !pushButtonState) goalSpeed -= 0.1;
        goalSpeed = constrain(goalSpeed, 0, 4);
        speedSetting = 22.0 * goalSpeed; //feed forward
      }
      if (courseSettingTimer > courseSetTime) {
        courseSettingTimer = 0;
        if (leftButtonState && !pushButtonState) {
          goalAngle -= 1;
          turnSetting = -60;
          leftTurn = false;
          rightTurn = false;
        }
        else if (leftButtonState && pushButtonState) {
          goalAngle = ekfYawAngle;
          turnTimer = 0;
          leftTurn = true;
          rightTurn = false;
        }
        else if (rightButtonState && !pushButtonState) {
          goalAngle += 1;
          turnSetting = 60;
          leftTurn = false;
          rightTurn = false;
        }
        else if (rightButtonState && pushButtonState) {
          goalAngle = ekfYawAngle;
          turnTimer = 0;
          leftTurn = false;
          rightTurn = true;
        }
      }
      if (turnTimer > 1000){
        turnTimer = 0;
      
        if (rightTurn ) { //right turn slowly for 180 degrees at 1 deg/sec
          goalAngle += turnRate ;
          turnSetting = 10; //feed forward
          
        }
        else if (leftTurn) { 
          goalAngle -= turnRate ;
          turnSetting = -10; //feed forward
        }
      }
      
      calculateMotorOutput();


    }
    else
    {
      speedSetting = 0;
      goalAngle = ekfYawAngle;
      rightMotor = stopMotorValue;
      leftMotor = stopMotorValue;
      leftTurn = false;
      rightTurn = false;
    }

    if (upButtonState && pushButtonState) {
      mode1started = true;
      goalAngle = ekfYawAngle;
      memset(differenceList, 0, sizeof(differenceList)) ;
      leftTurn = false;
      rightTurn = false;
      turnSetting = 0;
      
    }
    else if (downButtonState && pushButtonState) {
      memset(differenceSpeedList, 0, sizeof(differenceSpeedList));
      goalSpeed = 0;
      goalAngle = ekfYawAngle;
      leftTurn = false;
      rightTurn = false;
    }

  }
  //##############################################################################################
  //# Mode 2: Turn 90
  //##############################################################################################
  //##############################################################################################
  /*
    else if (mode == 2) { // turn 90 degrees
     mode = 4; //Skip ahead

     displayFigure8();

     if (mode2started)
     {
       if (speedSettingTimer > speedSetTime) {
         speedSettingTimer = 0;
         if (upButtonState && !pushButtonState) goalSpeed += 0.1; //mph
         if (downButtonState && !pushButtonState) goalSpeed -= 0.1;
         goalSpeed = constrain(goalSpeed, 0, 4);
         speedSetting = 22.0 * goalSpeed; //feed forward
       }
       if (courseSettingTimer > 1000) {
         courseSettingTimer = 0;
         if (rightButtonState) {
           turnGoal +=90;
           rightTurn = true;
         }
         if (leftButtonState)
         {
           turnGoal -=90;
           leftTurn = true;
         }
         if (turnGoal > 360) turnGoal -= 360;
         if (turnGoal < 0   ) turnGoal += 360;

         if (leftTurn) goalAngle -= 1;
         if (rightTurn) goalAngle += 1;
         if (goalAngle > 360) goalAngle -= 360;
         if (goalAngle < 0   ) goalAngle += 360;

         if (abs(goalAngle - turnGoal) < 1){
           leftTurn = false;
           rightTurn = false;
         }
       }
       calculateMotorOutput();


     }
     else
     {
       speedSetting = 0;
       goalAngle = ekfYawAngle;
       rightMotor = stopMotorValue;
       leftMotor = stopMotorValue;
     }

     if (upButtonState && pushButtonState) {
       mode2started = true;
       //speedSetting = 0;
       goalAngle = ekfYawAngle;
       turnGoal=ekfYawAngle;
       memset(differenceList, 0, sizeof(differenceList)) ;

     }
     if (downButtonState && pushButtonState) {
       memset(differenceSpeedList, 0, sizeof(differenceSpeedList));
       speedSetting = 0;
       goalSpeed = 0;
     }

    }
  */
  

  //##############################################################################################
  //# Mode 2: Figure 8
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################

  else if (mode == 2) { // figure 8
    if (upButtonState && pushButtonState) {
      mode4started = true;
      fixPointLat = gps.location.lat();
      fixPointLon = gps.location.lng();
      
      if (needsToPrint) {
        needsToPrint = false;
        Serial.print("fixPointLat\t");
        Serial.println(fixPointLat, 10);
        Serial.print("fixPointLon\t");
        Serial.println(fixPointLon, 10);
      }

      goalSpeed = 2.0;
      goalAngle = ekfYawAngle;
      memset(differenceList, 0, sizeof(differenceList));
      fig8phase = 0;
      fig8timer = 0;
    }


    displayFigure8();
    distanceToFixPoint = gps.distanceBetween(gps.location.lat(), gps.location.lng(), fixPointLat, fixPointLon);
    courseToFixPoint = gps.courseTo(gps.location.lat(), gps.location.lng(), fixPointLat, fixPointLon);

    if (mode4started) {
      if (speedSettingTimer > speedSetTime) {
        speedSettingTimer = 0;
        if (upButtonState && !pushButtonState) goalSpeed += 0.1; //mph
        else if (downButtonState && !pushButtonState) goalSpeed -= 0.1;
        goalSpeed = constrain(goalSpeed, 0, 4);
        speedSetting = 22.0 * goalSpeed; //feed forward
      }
      
      if (fig8phase == 0) { //forward for some seconds
        if (fig8timer >= 80000) {
          fig8timer = 0;
          fig8phase = 1;
          startAngle = ekfYawAngle; //degrees
        }
      }
      else if (fig8phase == 1) { //right turn slowly for 180 degrees at 1 deg/sec
        if (fig8timer >= 180000) {
          fig8timer = 0;
          fig8phase = 2;
          turnSetting = 0;
          startAngle = goalAngle;
        }
        else {
          goalAngle = startAngle + fig8timer / 1000.0; //turn rate of 1 deg/second
          turnSetting = 10; //feed forward
        }
      }
      else if (fig8phase == 2) { //Correct towards fixed point
        if (distanceToFixPoint > 10) {
          double angleDifference = courseToFixPoint - goalAngle;
          if (fig8timer >= 100) {
            fig8timer = 0;
            goalAngle += 0.1 * angleDifference / abs(angleDifference); //1 degree per second in the direction make the angle difference smaller
          }
        }
        else { //go straight once within 10 meters.

          fig8timer = 0;
          fig8phase = 3;
        }
      }
      else if (fig8phase == 3) { //go straight for 120 seconds
        if (fig8timer >= 100000) {
          fig8timer = 0;
          fig8phase = 4;
          startAngle = ekfYawAngle; //degrees
        }
      }
      else if (fig8phase == 4) { //right turn slowly for 180 degrees
        if (fig8timer >= 180000) {
          fig8timer = 0;
          fig8phase = 5;
          turnSetting = 0;
          startAngle = goalAngle;
        }
        else {
          goalAngle = startAngle - fig8timer / 1000.0; //turn rate of 1 deg/second (in opposite direction)
          turnSetting = -10; //feed forward
        }
      }
      else if (fig8phase == 5) { //Correct towards fixed point
        if (distanceToFixPoint > 10) {
          double angleDifference = courseToFixPoint - goalAngle;
          if (fig8timer >= 100) {
            goalAngle += 0.1 * angleDifference / abs(angleDifference); //1 degree per second in the direction make the angle difference smaller
            fig8timer = 0;
          }
        }
        else {//go straight once within 10 meters.
          fig8timer = 0;
          fig8phase = 6;
        }
      }
      else if (fig8phase == 6) { //forward for 20 seconds
        if (fig8timer >= 20000) {
          fig8timer = 0;
          fig8phase = 0;
        }
      }
      else { //nofig8phase
        Serial.println("No fig 8 phase... This is a problem.");
        delay(100);
        rightMotor = stopMotorValue;
        leftMotor = stopMotorValue;
      }

      calculateMotorOutput();
    }

    else //not mode4started
    {
      rightMotor = stopMotorValue;
      leftMotor = stopMotorValue;
    }
  }
  //##############################################################################################
  //# Mode 3: Sharp Turns
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################

  else if (mode == 3) { //Full
    if (upButtonState && pushButtonState) mode5started = true;
    displayFullSpeed();
    debugData();
    if (mode5started) {
      debugData();
      if (upButtonState) {
        rightMotor = int( 0.8 * maxFwdMotorValue * biasSetting);
        leftMotor = int(0.8 * maxFwdMotorValue);
      }
      else if (downButtonState) {
        rightMotor = int(0.8 * maxRevMotorValue) * biasSetting;
        leftMotor = int(0.8 * maxRevMotorValue);
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
  //# Mode 4: Calibrate Compass
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################

  else if (mode == 4) { //Calibrate
    resetCompassOffset(); // only runs once

    if (upButtonState && pushButtonState) {
      mode6started = true;
      compass.enterCalMode();
      delay(30);
      totalTurn = 0;
      lastAngle = compassHeading;
    }
    else if (upButtonState && !pushButtonState) //Try to reset the compassOffset
    {
      needsRealigned = true;
      resetCompassTimer = 0;
      headingCount = 0;
      compassOffset = 0;
      CANcompassOffset = 0;
      headingSum = 0;
      CANheadingSum = 0;
    }
    else if (downButtonState && pushButtonState) { //Write the current offset to memory
      EEPROM.put(compassOffsetAddress, compassOffset);
      EEPROM.put(CANcompassOffsetAddress, CANcompassOffset);
      compassHeading = getCompassHeading();
      ekf.setX(0, compassHeading);
    }
    else if (downButtonState && !pushButtonState) { //Reset the compass offset
      compassOffset = 0;
      CANcompassOffset = 0;
    }
    if (courseSettingTimer > courseSetTime) {
      courseSettingTimer = 0;
      if (leftButtonState) compassOffset -= 1;
      else if (rightButtonState) compassOffset += 1;
    }

    displayCompassCalibration();

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

        compass.exitCalMode();
        delay(50);
        compass.exitStandby();

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
  //# Mode 3: Anchor
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  /*
    else if (mode == 3) { // anchor
      mode = 4; //skip ahead

      if (upButtonState && pushButtonState) {
        mode3started = true;
        fixPointLat = gps.location.lat();
        fixPointLon = gps.location.lng();

      }


      displayAnchor();
     if (mode3started) {
        if (anchorAdjustTimer > 300){
            anchorAdjustTimer = 0;
            if (upButtonState ) {
               fixPointLat +=  (10.0/111030.0)*sin(radians(ekfYawAngle));
               fixPointLon += (10.0/111030.0)*cos(radians(ekfYawAngle));
            }
            else if (downButtonState ) {
               fixPointLat -= (10.0/111030.0)*sin(radians(ekfYawAngle));
               fixPointLon -= (10.0/80000.0)*cos(radians(ekfYawAngle));
            }
            if (leftButtonState ) {
               fixPointLat += (10.0/111030.0)*cos(radians(ekfYawAngle));
               fixPointLon += (10.0/80000.0)*sin(radians(ekfYawAngle));
            }
            else if (rightButtonState ) {
               fixPointLat -= (10.0/111030.0)*cos(radians(ekfYawAngle));
               fixPointLon -= (10.0/80000.0)*sin(radians(ekfYawAngle));
            }
        }


        distanceToFixPoint = gps.distanceBetween(gps.location.lat(), gps.location.lng(), fixPointLat, fixPointLon);
        courseToFixPoint = gps.courseTo(gps.location.lat(), gps.location.lng(), fixPointLat, fixPointLon);

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

  */

/*
  //##############################################################################################
  //# Mode 7: Frequency Sweep
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################
  //##############################################################################################

  else if (mode == 7) { //Calibrate with a frequecy sweep
    if (upButtonState && pushButtonState) {
      mode7started = true;
      sineSweepTimer = 0;

    }

    displayFrequencySweeps();

    if (mode7started) {

      while (sineSweepTimer <= 10000) //number of milliseconds
      {
        getMeasurements();
        sendCANmessages();
        debugData();
        displayData();
        displayFrequencySweeps();

        rightMotor = stopMotorValue + 50; //go straight for 10 seconds
        leftMotor = stopMotorValue + 50;

        rightServo.write(rightMotor);
        leftServo.write(leftMotor);

        if (mode != 7) {//abort
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
        displayFrequencySweeps();

        omega =  sineSweepTimer / 500000.0;
        motorInput = 50 * sin( 2 * omega * PI * sineSweepTimer / 1000.);
        rightMotor = stopMotorValue + 50 + int(motorInput);
        leftMotor = stopMotorValue + 50 - int(motorInput);

        rightServo.write(rightMotor);
        leftServo.write(leftMotor);

        if (mode != 7) { //abort
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
*/

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

////////////////////////////////////////////////////////////////////////////////////////
void resetOutputs() {
  speedSetting = 0;
  mode1started = false;
  mode2started = false;
  mode3started = false;
  mode4started = false;
  mode5started = false;
  mode6started = false;
  mode7started = false;
  needsToPrint = true;

  compass.exitStandby();

  //tft.fillScreen(ILI9341_GREEN);
  delay(50);
  //displayTemplate();
  currentMode = mode;

  //reset the integrator
  memset(differenceList, 0, sizeof(differenceList)) ;
  memset(differenceSpeedList, 0, sizeof(differenceSpeedList)) ;
  turnSetting = 0;
  angleSetting = 0;
  rightMotor = stopMotorValue;
  leftMotor  = stopMotorValue;
}
////////////////////////////////////////////////////////////////////////////////////////


/***************************************************************************************/
/*Display Modes*************************************************************************/

void displayDefault() {
  if (modeDisplayTimer >= modeDisplayPeriod) {
    modeDisplayTimer = 0;
    sprintf(topLine, "OFF O:%5.1f N:%2i", compassOffset, int(gps.satellites.value()));
    displayTopLine(topLine);
    sprintf(botLine, "H:%3i C:%3i S%2.1f", int(ekfYawAngle), int(gps.course.deg()), ekfSpeed);
    displayBottomLine(botLine);
  }
}

void displayNormalOperation() {
  if (modeDisplayTimer >= modeDisplayPeriod) {
    modeDisplayTimer = 0;
    sprintf(topLine, "%i Manual Spd:%3.1f", mode, goalSpeed);
    displayTopLine(topLine);
    if (mode1started) sprintf(botLine, "H:%3i G:%3i S%2.1f", int(ekfYawAngle), int(goalAngle), ekfSpeed);
    else strncpy(botLine, "Butn+Up to Start", 16);
    displayBottomLine(botLine);
  }
}

void displayFigure8() {
  if (modeDisplayTimer >= modeDisplayPeriod) {
    modeDisplayTimer = 0;
    sprintf(topLine, "Fig8 S:%3.1f D:%3i", goalSpeed, int(distanceToFixPoint));
    displayTopLine(topLine);
    if (mode4started) sprintf(botLine, "H:%3i G:%3i S%3.1f", int(ekfYawAngle), int(goalAngle), ekfSpeed);
    else strncpy(botLine, "Butn+Up to Start", 16);
    displayBottomLine(botLine);
  }
}

void displayAnchor() {
  if (modeDisplayTimer >= modeDisplayPeriod) {
    modeDisplayTimer = 0;
    sprintf(topLine, "%i Anch C%3i D%3i", mode, int(courseToFixPoint), int(distanceToFixPoint));
    displayTopLine(topLine);
    if (mode3started) sprintf(botLine, "H:%3i G:%3i S%2.1f", int(ekfYawAngle), int(goalAngle), ekfSpeed);
    else strncpy(botLine, "Butn+Up to Start", 16);
    displayBottomLine(botLine);
  }
}

void displayTurn90() {
  if (modeDisplayTimer >= modeDisplayPeriod) {
    modeDisplayTimer = 0;
    sprintf(topLine, "%i Turn90 Spd:%3.1f", mode, goalSpeed);
    displayTopLine(topLine);
    if (mode2started) sprintf(botLine, "H:%3i G:%3i S%2.1f", int(ekfYawAngle), int(goalAngle), ekfSpeed);
    else strncpy(botLine, "Butn+Up to Start", 16);
    displayBottomLine(botLine);
  }
}

void displayFullSpeed() {
  if (modeDisplayTimer >= modeDisplayPeriod) {
    modeDisplayTimer = 0;
    sprintf(topLine, "%i Full Spd:%4.1f", mode, ekfSpeed); //motorInput is the value sent to the motors.
    displayTopLine(topLine);
    if (mode5started) sprintf(botLine, "H:%3i YawRt:%4.1f", int(ekfYawAngle), ekfYawRate);
    else strncpy(botLine, "Butn+Up to Start", 16);
    displayBottomLine(botLine);
  }
}

void displayCompassCalibration() {
  if (modeDisplayTimer >= modeDisplayPeriod) {
    modeDisplayTimer = 0;
    sprintf(topLine, "Cal. O:%3i H:%3i", int(compassOffset), int(compassHeading)); //motorInput is the value sent to the motors.
    displayTopLine(topLine);
    if (mode6started) sprintf(botLine, "Turn:%4i Hdg:%3i", int(totalTurn), int(gps.course.deg()));
    else sprintf(botLine, "BtnUp2Start C%3i", int(gps.course.deg()));
    displayBottomLine(botLine);
  }
}

void displayFrequencySweeps() {
  if (modeDisplayTimer >= modeDisplayPeriod) {
    modeDisplayTimer = 0;
    sprintf(topLine, "%i Freq. Sweep %2i", mode, int(motorInput)); //motorInput is the value sent to the motors.
    displayTopLine(topLine);
    if (mode7started) sprintf(botLine, "%5.4f H:%3i S:%2.1f", omega, int(ekfYawAngle), ekfSpeed);
    else strncpy(botLine, "Butn+Up to Start", 16);
    displayBottomLine(botLine);
  }
}
/*End Display Modes*************************************************************************/
/***************************************************************************************/



/*****************************************************************************************/
/* DISPLAY HELPER FUNCTIONS */
void displayUpperLeft8(char message[9]) {
  if (upperLeftTimer > 100){
    upperLeftTimer = 0;
    txmsg.id = 0x211; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    Can0.write(txmsg);
  }
}

void displayUpperRight8(char message[9]) {
  if (upperRightTimer > 100){
    upperRightTimer = 0;
    txmsg.id = 0x212; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    Can0.write(txmsg);
  }
}

void displayLowerLeft8(char message[9]) {
  if (lowerLeftTimer > 100){
    lowerLeftTimer = 0;
    txmsg.id = 0x221; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    Can0.write(txmsg);
  }
}


void displayLowerRight8(char message[9]) {
  if (lowerRightTimer > 100){
    lowerRightTimer = 0;
    txmsg.id = 0x222; //sent to the lower right
    for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
    Can0.write(txmsg);
  }
}

void displayTopLine(char _topLine[17]) {
  char message[9];
  for (int j = 0; j < 8; j++) message[j] = _topLine[j];
  displayUpperLeft8(message);
  for (int j = 8; j < 16; j++) message[j - 8] = _topLine[j];
  displayUpperRight8(message);
}

void displayBottomLine(char _botLine[17]) {
  char message[9];
  for (int j = 0; j < 8; j++) message[j] = _botLine[j];
  displayLowerLeft8(message);
  for (int j = 8; j < 16; j++) message[j - 8] = _botLine[j];
  displayLowerRight8(message);
}
/* END DISPLAY HELPER FUNCTIONS */
/*****************************************************************************************/




void  calculateMotorOutput() {
  if (goalAngle > 360) goalAngle -= 360;
  if (goalAngle < 0) goalAngle += 360;

  if (calculateMotorOutputTimer >= deltaTms) {
    calculateMotorOutputTimer = 0;

    difference = goalAngle - ekfYawAngle; //using Kalman Filter output for input
    if (difference <= -180)  difference += 360;
    if (difference >= 180)  difference -= 360;

    if (abs(turnSetting) < 15) { //Suspend while changing course to reduce integral wind-up. The turnSetting is a feed-forward variablet
      differenceList[diffIndex] = int32_t(difference * 1000);
      diffIndex += 1;
      if (diffIndex >= memorySize) diffIndex = 0;
    }

    int32_t sum = 0;
    for (int j = 0; j < memorySize; j++) {
      sum += differenceList[j];
    }

    integral = double(sum) / 1000.0 * deltaT;

    if (gps.speed.isUpdated())
    {
      speedDifference = goalSpeed - ekfSpeed;

      differenceSpeedList[diffIndex] = int32_t(speedDifference * 1000);
      diffSpeedIndex += 1;
      if (diffSpeedIndex >= memorySize) diffSpeedIndex = 0;

      int32_t speedSum = 0;
      for (int j = 0; j < memorySize; j++) speedSum += differenceSpeedList[j];

      double integralSpeed = double(speedSum) / 1000.0 * deltaT;

      speedAdjust = constrain(int(speedK * speedDifference + speedI * integralSpeed),-50,50);

    }

    angleSetting = constrain(int(angleK * difference + angleI * integral + angleD * ekfYawRate),-100,100);

  
    int tempRightMotor = int((speedSetting + speedAdjust - turnSetting - angleSetting + stopMotorValue) * biasSetting);
    int tempLeftMotor  = int( speedSetting + speedAdjust + turnSetting + angleSetting + stopMotorValue) ;
    rightMotor = constrain(tempRightMotor, maxRevMotorValue, maxFwdMotorValue);
    leftMotor  = constrain(tempLeftMotor, maxRevMotorValue, maxFwdMotorValue);


    //print
    debugData();
  }

}

