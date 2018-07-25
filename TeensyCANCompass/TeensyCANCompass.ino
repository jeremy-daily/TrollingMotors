/*
   The Teensy Troller
   A program to implement an autopilot with two trolling motors on a ski boat.

   By Jeremy Daily

   Released under the beer license: If you like it, I'd like to share a beer with you if we ever meet.
*/
#include <TinyGPS++.h> // Used to read the data from the GPS.
#include <FlexCAN.h> // The CAN library for the Teensy used to communicate with the Joystick

elapsedMillis broadcastCANtimer; //set up intervals

//Initialize the GPS
TinyGPSPlus gps;

//Set up CAN messaging
FlexCAN CANbus();
static CAN_message_t txmsg;
uint32_t CANTXcount = 0;

boolean LEDstate;

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  delay(10);
  Serial.println("Starting CAN at 500k... ");
  Can0.begin(500000);
  Serial.println("Done.");
  
  Serial.print("Starting GPS... ");
  Serial1.begin(57600);
  Serial1.println("$PMTK220,200*2C"); //update at 5 Hz
  delay(100);
  Serial1.println("$PMTK300,200,0,0,0,0*2F"); //position fix update to 5 Hz
  Serial.println("Done.");
}

void sendCANmessages() {
  if (broadcastCANtimer >= 200) {
    broadcastCANtimer = 0;
    LEDstate = !LEDstate;
    digitalWrite(LED_BUILTIN,LEDstate);
    //GPS Messages
    if (gps.speed.isUpdated() ) {
      txmsg.id = 0x45e;
      txmsg.len = 8;
      Serial.print("GPS Speed: ");
      Serial.println(gps.speed.value());
      Serial.print("GPS Course: ");
      Serial.println(gps.course.value());
      
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

      txmsg.buf[0] = byte( (gps.location.rawLat().billionths & 0xFF000000) >> 24);
      txmsg.buf[1] = byte( (gps.location.rawLat().billionths & 0x00FF0000) >> 16);
      txmsg.buf[2] = byte( (gps.location.rawLat().billionths & 0x0000FF00) >>  8);
      txmsg.buf[3] = byte( (gps.location.rawLat().billionths & 0x000000FF) >>  0);
      txmsg.buf[4] = byte( (gps.location.rawLng().billionths & 0xFF000000) >> 24);
      txmsg.buf[5] = byte( (gps.location.rawLng().billionths & 0x00FF0000) >> 16);
      txmsg.buf[6] = byte( (gps.location.rawLng().billionths & 0x0000FF00) >>  8);
      txmsg.buf[7] = byte( (gps.location.rawLng().billionths & 0x000000FF) >>  0);
      Serial.print("GPS lat: ");
      Serial.print(gps.location.rawLat().negative ? "-" : "+");
      Serial.print(gps.location.rawLat().deg);
      Serial.print(".");
      Serial.println(gps.location.rawLat().billionths);
      Serial.print("GPS lng: ");
      Serial.print(gps.location.rawLng().negative ? "-" : "+");
      Serial.print(gps.location.rawLng().deg);
      Serial.print(".");
      Serial.println(gps.location.rawLng().billionths);
      
      Can0.write(txmsg);
      CANTXcount++;
    }

  }
}


void getMeasurements() {
  
   
  while (Serial1.available()){
    char c = Serial1.read();
    gps.encode(c);
    //Serial.write(c);
    
  }
}

void loop() {

  getMeasurements();

  //send stuff
  sendCANmessages();
}

