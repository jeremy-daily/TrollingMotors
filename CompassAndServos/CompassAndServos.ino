/*
   The Teensy Troller
   A program to implement an autopilot with two trolling motors on a ski boat.

   By Jeremy Daily

   Released under the beer license: If you like it, I'd like to share a beer with you if we ever meet.
*/
#include <TinyGPS++.h> // Used to read the data from the GPS.
#include <FlexCAN.h> // The CAN library for the Teensy used to communicate with the Joystick
#include <i2c_t3.h> // the I2C library that replaces Wire.h for the Teensy 3.2
#include <SFE_HMC6343.h> //Sparkfun's library for the digital compass
#include <Servo.h> // Used to send pulses to the motor controller
#include <EEPROM.h> //used to store compass declination angles and calibration results
#include "BNO055.h"

#define compassOffsetAddress 0
#define CANcompassOffsetAddress 8
double compassOffset; // True - measured, so measured + offset = true

const uint32_t deltaTms = 50; //milliseconds for calculations and output

elapsedMillis broadcastCANtimer; //set up intervals
elapsedMillis waitingForCANtimer;
elapsedMillis debugSerialtimer;
elapsedMillis CANaliveTimer;
elapsedMillis SerialAliveTimer;
elapsedMillis broadcastCANmodeTimer;
elapsedMillis gyroReadingTimer;

double tempHeading = 0;
double compassHeading = 0.0;

const int stopMotorValue = 92;
const int maxRevMotorValue = 8;
const int maxFwdMotorValue = 208;
uint8_t leftMotor = 92;
uint8_t rightMotor = 92;
uint8_t tempLeftMotor = 92;
uint8_t tempRightMotor = 92;

//Initialize the GPS
TinyGPSPlus gps;

SFE_HMC6343 compass; // Declare the sensor object

Servo rightServo;  // create servo object to control a servo
Servo leftServo;  // create servo object to control a servo

static CAN_message_t txmsg, rxmsg;
uint32_t CANTXcount = 0;
uint32_t CANRXcount = 0;
uint32_t ID = 0;

void setup() {
  compassOffset = -23.15299213;
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
  Serial.println("Done with setting up the BNO055.");



  Serial.print("Starting Servos... ");
  rightServo.attach(16);  // attaches the servo on pin 16 to the servo object
  leftServo.attach(23);  // attaches the servo on pin 23 to the servo object
  Serial.println("Done.");

  Serial.print("Starting GPS... ");
  Serial1.begin(9600);
  Serial.println("Done.");
  delay(300);
  Serial.println("Setting GPS to 57600 baud... ");
  Serial1.println("$PMTK251,57600*2C"); //Set Baud Rate to 57600
  delay(100);
  Serial1.flush();
  Serial1.end();
  Serial1.begin(57600);
  Serial.println("Done.");
  
  Serial.println("Setting GPS to update at 5 Hz... ");
  Serial1.println("$PMTK220,200*2C"); //update at 5 Hz
  delay(100);
  Serial1.println("$PMTK300,200,0,0,0,0*2F"); //position fix update to 5 Hz
  Serial.println("Done.");


  Serial.println("Starting Compass... ");
  compass.init();
  byte bSN_LSB = compass.readEEPROM(SN_LSB);
  byte bSN_MSB = compass.readEEPROM(SN_MSB);
  Serial.println("Done.");
  
  Serial.print("Compass Serial Number: ");
  Serial.println(word(bSN_MSB, bSN_LSB));
  compass.writeEEPROM(0x14, 0x00); //Set filter to 0.
  compass.exitStandby();
  delay(100);

  Serial.println("Starting CAN at 250k... ");
  Can0.begin(250000);
  Serial.println("Done.");

  Serial.println("Loading Compass Offsets from EEPROM... ");
  EEPROM.get(compassOffsetAddress, compassOffset);
  Serial.println("Done.");
  delay(1500);
  while (Serial1.available()) gps.encode(Serial1.read());
  delay(50);
  debugDataHeader();
}


void sendCANmessages() {
  if (broadcastCANtimer >= deltaTms) {
    broadcastCANtimer = 0;

    //GPS Messages
    if (gps.speed.isUpdated() ) {
      txmsg.id = 0x43e; //status message
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
    uint16_t yawRate_int = yawRate * 1000 + 32768;
    broadcastCANmodeTimer = 0;
    txmsg.id = 0x701; //Status message
    txmsg.len = 8;
    txmsg.buf[0] = rightMotor;
    txmsg.buf[1] = leftMotor;
    txmsg.buf[2] = byte( (compass.heading & 0xFF00) >>  8);
    txmsg.buf[3] = byte( (compass.heading & 0x00FF) >>  0);
    txmsg.buf[4] = byte( (yawRate_int     & 0xFF00) >>  8);
    txmsg.buf[5] = byte( (yawRate_int     & 0x00FF) >>  0);
    txmsg.buf[6] = byte( (millis()        & 0xFF00) >>  8);
    txmsg.buf[7] = byte( (millis()        & 0x00FF) >>  0);   
    Can0.write(txmsg);
    CANTXcount++;
  }
}

void readCANMessages( ) {
  Can0.read(rxmsg);
  waitingForCANtimer = 0; //reset the can message timeout
  ID = rxmsg.id;
  if (ID == 0x201) {
    CANaliveTimer = 0;
    tempRightMotor = rxmsg.buf[0];
    tempLeftMotor  = rxmsg.buf[1];
  } 
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

 
  Serial.print("Time [ms]\t");
  Serial.print("Compass [deg]\t");
  Serial.print("GPS Course [deg]\t");
  Serial.print("GPS Speed [mph]\t");
  Serial.print("Yaw Rate [deg/s]\t");
  Serial.print("rightMotor\t");
  Serial.print("leftMotor\t");
  Serial.print("GPS Lat [deg]\t");
  Serial.print("GPS Long[deg]\n");
  
 
}

void debugData() {
  if (debugSerialtimer >= deltaTms) {
    debugSerialtimer = 0;

    Serial.print(millis());
    Serial.print("\t");
    Serial.print(compass.heading/10.0);
    Serial.print("\t");
    Serial.print(gps.course.deg());
    Serial.print("\t");
    Serial.print(gps.speed.mph());
    Serial.print("\t");
    Serial.print(yawRate);
    Serial.print("\t");
    Serial.print(rightMotor);
    Serial.print("\t");
    Serial.print(leftMotor);
    Serial.print("\t");
    Serial.print(gps.location.lat(), 8);
    Serial.print("\t");
    Serial.println(gps.location.lng(), 8);
  }
}

double getCompassHeading() {
  compass.readHeading();
  if ((int(gps.satellites.value()) > 7) && (gps.speed.mph() > 1.0) ){
    tempHeading = gps.course.deg();
  }
  else {
    tempHeading = compass.heading / 10.0 + compassOffset; //local compass
  }
  
  return tempHeading;
}


void getMeasurements() {
  //measure stuff
  if (gyroReadingTimer >= deltaTms) {
    gyroReadingTimer = 0;

    yawRate = BNOgetYawRate();
    compassHeading = getCompassHeading();
  }
  while (Serial1.available()) gps.encode(Serial1.read());
}

void loop() {
  //read stuff
  getMeasurements();

  //send stuff
  sendCANmessages();
  
  //get user input
  readCANMessages();
  
  if (CANaliveTimer > 200) {
    rightMotor = stopMotorValue;
    leftMotor = stopMotorValue;
  }
  else {
    rightMotor = constrain(tempRightMotor, maxRevMotorValue, maxFwdMotorValue);
    leftMotor  = constrain(tempLeftMotor,  maxRevMotorValue, maxFwdMotorValue);
  }
  
  //always send the updates to the motors
  rightServo.write(rightMotor);
  leftServo.write(leftMotor);

  //print stuff
  debugData();
}
