/*
   The Teensy CAN Compass
   A program to implement an autopilot with two trolling motors on a ski boat.

   By Jeremy Daily

   Released under the beer license: If you like it, I'd like to share a beer with you if we ever meet.
*/
#include <TinyGPS++.h> // Used to read the data from the GPS.
#include <FlexCAN.h> // The CAN library for the Teensy used to communicate with the Joystick
#include  <i2c_t3.h> // the I2C library that replaces Wire.h for the Teensy 3.2
//#include <SFE_HMC6343.h> //Sparkfun's library for the digital compass
#include <EEPROM.h> //used to store compass declination angles and calibration results

elapsedMillis broadcastCANtimer; //set up intervals
elapsedMillis resetCompassTimer;

#define compassOffsetAddress 0
#define CANcompassOffsetAddress 8
float compassOffset = -10.8;// True - measured, so measured + offset = true


//Initialize the GPS
TinyGPSPlus gps;
//SFE_HMC6343 compass; // Declare the sensor object
//Set up CAN messaging
FlexCAN CANbus();
static CAN_message_t txmsg;
uint32_t CANTXcount = 0;

boolean LEDstate;
boolean needsRealigned = true;

int compass_address = 0x21;
double tempHeading;
uint16_t headingReading;

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  Wire.begin();
  delay(100);
  Serial.println("Starting CAN at 500k... ");
  Can0.begin(500000);
  Serial.println("Done.");

  Serial.print("Starting GPS... ");
  Serial1.begin(9600);
  Serial.println("Done.");
  delay(300);
  Serial1.println("$PMTK251,57600*2C"); //Set Baud Rate to 57600
  delay(100);
  Serial1.flush();
  Serial1.end();
  Serial.println("Setting GPS to 57600 baud... ");
  Serial1.begin(57600);
  Serial1.println("$PMTK251,57600*2C"); //Set Baud Rate to 57600
  Serial.println("Done.");
  Serial1.println("$PMTK220,200*2C"); //update at 5 Hz
  delay(100);
  Serial1.println("$PMTK300,200,0,0,0,0*2F"); //position fix update to 5 Hz
  Serial.println("Done.");

  Serial.println("Loading Compass Offsets from EEPROM... ");
  EEPROM.get(compassOffsetAddress, compassOffset);
  Serial.println("Done.");
  delay(50);
}

uint32_t headingSum;
int headingCount;

void resetCompassOffset() {
  if (needsRealigned) {
    if (gps.speed.mph() > 15.0 && gps.speed.age() < 1000) //compass heading can be realigned to velocity vector
    {
      if (resetCompassTimer >= 200)
      {
        resetCompassTimer = 0;
        headingSum += get_heading()/10;
        headingCount += 1;
        Serial.print(headingCount);
        Serial.print("\t");
        Serial.println(headingSum);
      }

      if (headingCount >= 20) {
        needsRealigned = false;
        compassOffset =   gps.course.deg() - headingSum / headingCount;
        
        if (compassOffset < -180) compassOffset += 360;
        if (compassOffset > 180) compassOffset -= 360;

        Serial.print("Setting Compass Offsets Based on GPS heading: compassOffset = ");
        Serial.println(compassOffset);
        
        EEPROM.put(compassOffsetAddress, compassOffset);
        
        
      }
    }
  }
}

uint16_t getCompassHeading() {
  
 tempHeading = double(get_heading())/10 + compassOffset; //local compass
 
  if (tempHeading < 0) tempHeading += 360;
  if (tempHeading > 360) tempHeading -= 360;

  return uint16_t(tempHeading*10);
}


void sendCANmessages() {
  if (broadcastCANtimer >= 200) {
    broadcastCANtimer = 0;
    

    headingReading = getCompassHeading();
    txmsg.id = 0x43d;
    txmsg.len = 8;
    Serial.print("Compass Heading (tenths): ");
    Serial.println(headingReading);
    txmsg.buf[0] = 0xFF;
    txmsg.buf[1] = 0xFF;
    txmsg.buf[2] = 0xFF;
    txmsg.buf[3] = 0xFF;
    txmsg.buf[4] = 0xFF;
    txmsg.buf[5] = 0xFF;
    txmsg.buf[6] = byte(( headingReading & 0xFF00) >> 8);
    txmsg.buf[7] = byte( headingReading & 0x00FF);
    
    //GPS Messages
    txmsg.id = 0x43e;
    txmsg.len = 8;
    Serial.print("GPS Speed: ");
    Serial.println(gps.speed.value());
    Serial.print("GPS Course: ");
    Serial.println(gps.course.value());
    txmsg.buf[0] = byte( (headingReading & 0xFF00) >> 8);
    txmsg.buf[1] = byte( (headingReading & 0x00FF));
    txmsg.buf[2] = byte( (gps.speed.value() & 0x0000FF00) >>  8);
    txmsg.buf[3] = byte( (gps.speed.value() & 0x000000FF) >>  0);
    txmsg.buf[4] = byte( (gps.course.value() & 0x0000FF00) >>  8); //Hundredths
    txmsg.buf[5] = byte( (gps.course.value() & 0x000000FF) >>  0);
    txmsg.buf[6] = byte( (gps.satellites.value() & 0xFF));
    txmsg.buf[7] = byte( (gps.course.age()/100) & 0xFF);
    
    Can0.write(txmsg);
    CANTXcount++;
    int32_t latitude = gps.location.lat()*1000000;
    int32_t longitude = gps.location.lng()*1000000;
    
    txmsg.id = 0x46e;
    txmsg.buf[0] = byte( (latitude & 0xFF000000) >> 24);
    txmsg.buf[1] = byte( (latitude & 0x00FF0000) >> 16);
    txmsg.buf[2] = byte( (latitude & 0x0000FF00) >>  8);
    txmsg.buf[3] = byte( (latitude & 0x000000FF) >>  0);
    txmsg.buf[4] = byte( (longitude & 0xFF000000) >> 24);
    txmsg.buf[5] = byte( (longitude & 0x00FF0000) >> 16);
    txmsg.buf[6] = byte( (longitude & 0x0000FF00) >>  8);
    txmsg.buf[7] = byte( (longitude & 0x000000FF) >>  0);
    Serial.print("GPS lat: ");
    Serial.println(latitude);
    Serial.print("GPS lng: ");
    Serial.println(longitude);
  }
}


void getMeasurements() {
  
   
  while (Serial1.available()){
    char c = Serial1.read();
    gps.encode(c);
    //Serial.write(c);
    LEDstate = !LEDstate;
    digitalWrite(LED_BUILTIN,LEDstate);
  }
}

void loop() {

  getMeasurements();

  //send stuff
  sendCANmessages();
}

int get_heading()
{
  byte val = 0;
  byte data[2];
  int  j, frac;

  Wire.beginTransmission(0x21);
  Wire.send(0x41); //A
  Wire.endTransmission();
  delay(8); //6000 microseconds minimum 6 ms 

  Wire.requestFrom(0x21, 2);
  j = 0;
  while(Wire.available())
  {
    char c = Wire.receive();
    data[j] = c;
    j++;
  }
  frac = data[0]*256 + data[1];
  Serial.print("Compass frac: ");
   Serial.println(frac);
  return frac;
}
