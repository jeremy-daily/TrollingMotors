#include <Canbus.h>
#include <Wire.h>


#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false

int compassAddress = 0x42 >> 1;
byte headingData[2];
int headingValue = 0;


//Set up the pins for SPI
#define	P_MOSI	      B,3 //Pin 11
#define	P_MISO	      B,4 //Pin 12
#define	P_SCK	      B,5 //Pin 13
#define	MCP2515_CS    B,2 //Pin 10
#define	MCP2515_INT   D,2 //Pin 2

#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>




//Initialize Timing varialbles
unsigned long previousMillis1000 = 0;
unsigned long previousMillis200 = 0;
unsigned long previousMillis20 = 0;


void setup()
{ 

  
  Wire.begin();
  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
  useInterrupt(true);
  
  //Initialize MCP2515 CAN controller at the specified speed
  if(Canbus.init(CANSPEED_500)) 
    Serial.println("CAN Init ok");
  else 
    Serial.println("Can't init CAN");
 
  
}
 
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    boolean usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    boolean usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop()
{
  
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  tCAN message;
   
  unsigned long currentMillis = millis();
  
  
  if(currentMillis - previousMillis200 > 50) { //Perform this every 100 milliseconds
    previousMillis200 = currentMillis;   
    
     // Send a "A" command to the HMC6352
      // This requests the current heading data
      Wire.beginTransmission(compassAddress);
      Wire.write("A");              // The "Get Data" command
      Wire.endTransmission();
      delay(1);                   // The HMC6352 needs at least a 70us (microsecond) delay
      // after this command.  Using 10ms just makes it safe
      // Read the 2 heading bytes, MSB first
      // The resulting 16bit word is the compass heading in 10th's of a degree
      // For example: a heading of 1345 would be 134.5 degrees
      Wire.requestFrom(compassAddress, 2);        // Request the 2 byte heading (MSB comes first)
      int i = 0;
      while(Wire.available() && i < 2)
      { 
        headingData[i] = Wire.read();
        i++;
      }
      headingValue = headingData[0]*256 + headingData[1];  // Put the MSB and LSB together
      //Serial.print("Current heading: ");
      Serial.print(int (headingValue / 10));     // The whole number part of the heading
      Serial.print(".");
      Serial.println(int (headingValue % 10));     // The fractional part of the heading
      //Serial.println(" degrees");
    
    
    message.id = 0x43C; //Made up Broadcast message
    message.header.rtr = 0;
    message.header.length = 2;
    message.data[0] = headingData[0];
    message.data[1] = headingData[1]; 
    
                   
    mcp2515_send_message(&message);
  }
  
  if(currentMillis - previousMillis1000 > 1000){
    previousMillis1000=currentMillis;
    message.id = 0x43E; //Made up Broadcast message
    message.header.rtr = 0;
    message.header.length = 8;
    message.data[0] = headingData[0];
    message.data[1] = headingData[1]; 
    message.data[2] = highByte(int(GPS.speed)); 
    message.data[3] = lowByte(int(GPS.speed)); 
    message.data[4] = highByte(int(GPS.angle)); 
    message.data[5] = lowByte(int(GPS.angle)); 
    message.data[6] = lowByte(int(GPS.satellites)); 
    message.data[7] = lowByte(int(GPS.fix)); 
                   
    mcp2515_send_message(&message);
  
    byte * latBuffer = (byte *) &GPS.latitude;
    byte * lonBuffer = (byte *) &GPS.longitude;
//    byte latBuffer[4];
//      latBuffer[0] = highByte(int(GPS.latitude));
//      latBuffer[1] = lowByte(int(GPS.latitude));
//      latBuffer[2] = highByte(int(GPS.lat));
//      latBuffer[3] = lowByte(int(GPS.lat));
    
//    byte lonBuffer[4];
//      lonBuffer[0] = highByte(int(GPS.longitude));
//      lonBuffer[1] = lowByte(int(GPS.longitude));
//      lonBuffer[2] = highByte(int(GPS.lon));
//      lonBuffer[3] = lowByte(int(GPS.lon));  
  
    message.id = 0x43F; //Made up Broadcast message
    message.header.rtr = 0;
    message.header.length = 8;
    message.data[0] = latBuffer[0];
    message.data[1] = latBuffer[1]; 
    message.data[2] = latBuffer[2]; 
    message.data[3] = latBuffer[3]; 
    message.data[4] = lonBuffer[0]; 
    message.data[5] = lonBuffer[1]; 
    message.data[6] = lonBuffer[2]; 
    message.data[7] = lonBuffer[3]; 
                   
    mcp2515_send_message(&message);
  
    //Serial.print("Voltage Sense: ");
    //Serial.println(vSenseReading);
  }
  
 
}
