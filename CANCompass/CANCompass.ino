    
#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <Wire.h>
//required for fmod()
#include <math.h>;

// enable the CAN interface with the MCP2515 chip
MCP_CAN CAN0(10); 

const int headingOffset = -20; //Compensates for the mount of the compass in the case. Tenths

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false

int compassAddress = 0x42 >> 1;
byte headingData[2];
int headingValue = 0;

byte data[8];

//CAN interface messages (Borrowed from the example).
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];


double lon;
double lat;

//Initialize Timing varialbles
unsigned long previousMillis1000 = 0;
unsigned long previousMillis200 = 0;
unsigned long previousMillis20 = 0;

unsigned long currentMillis =0;

double x2lat=  0   ;  //enter a latitude point here   this is going to be your waypoint
double x2lon=  0    ;  // enter a longitude point here  this is going to be your waypoint

double dist_calc=0;
double dist_calc2=0;
double diflat=0;
double diflon=0;
int headerValue=0;
byte mode;
byte buttons;
byte headingValueArray[2];

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
   Serial.println("Setting up CAN0..."); //J1939
  if(CAN0.begin(CAN_500KBPS) == CAN_OK) Serial.println("CAN0 init ok!!");
  else Serial.println("CAN0 init fail!!");

    Serial.print("Millis");
    Serial.print("\t");
    Serial.print("Head");
    Serial.print("\t");
    Serial.print("Speed");
    Serial.print("\t");
    Serial.print("Angle");
    Serial.print("\t");
    Serial.print("Sats");
    Serial.print("\t");
    Serial.print("Fix");
    Serial.print("\t");
    Serial.print("Quality");
    Serial.print("\t");
    Serial.print("Long");
    Serial.print("\t");
    Serial.print("Long (dec)");
    Serial.print("\t");
    Serial.print("Lat");
    Serial.print("\t");
    Serial.print("Lat (dec)");
    Serial.print("\t");
    Serial.print("Alt");
    Serial.print("\t");
    Serial.print("Date");
    Serial.print("\t");
    Serial.print("Time");
    Serial.print("\t");
    Serial.print("Year");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.print(GPS.milliseconds);
    Serial.print("\t");
    Serial.print("Fixed Lon");
    Serial.print("\t");
    Serial.print("Fixed Lat");
    Serial.print("\t");
    Serial.print("Distance");
    Serial.print("\t");
    Serial.println("Goal");

    previousMillis1000 = millis();
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

void readCANbus(){
  CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
  rxId = CAN0.getCanId();                    // Get message ID
   if (rxId == 0x777){
     mode = rxBuf[0];
     buttons = rxBuf[1];
   }
   if (mode == 3 && bitRead(buttons, 4)){ //Fix
    x2lat=lat;
    x2lon=lon;
   }
   if (mode == 4 && bitRead(buttons, 4)){ //Figure 8
    x2lat=lat;
    x2lon=lon;
   }
   
}

void loop()
{
  readCANbus();
  
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  currentMillis = millis();
   
     
  if(currentMillis - previousMillis200 >= 100) { //Perform this every 100 milliseconds
    previousMillis200 = currentMillis;   
     
     // Send a "A" command to the HMC6352
      // This requests the current heading data
       Wire.beginTransmission(compassAddress);
      Wire.write("A");              // The "Get Data" command
      Wire.endTransmission();
      
      
      delay(2);                   // The HMC6352 needs at least a 70us (microsecond) delay
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
      headingValue = int(headingData[0])*256 + int(headingData[1]);  // Put the MSB and LSB together
      
      if (headingValue >3600) headingValue -= 3600;
      if (headingValue <0) headingValue += 3600;
      
      headingValueArray[0]=highByte(headingValue);
      headingValueArray[1]=lowByte(headingValue);
      
      CAN0.sendMsgBuf(0x43c, 0, 2, headingValueArray );
    
      

  }
  currentMillis = millis();
  if(currentMillis - previousMillis1000 >= 1000){
    previousMillis1000=currentMillis-100;
  
    lon = convertDegMinToDecDeg(GPS.longitude);
    lat = convertDegMinToDecDeg(GPS.latitude);
  
     data[0] = highByte(headingValue);
     data[1] = lowByte(headingValue); 
     data[2] = highByte(int(GPS.speed)); 
     data[3] = lowByte(int(GPS.speed)); 
     data[4] = highByte(int(GPS.angle)); 
     data[5] = lowByte(int(GPS.angle)); 
     data[6] = lowByte(int(GPS.satellites)); 
     data[7] = lowByte(int(GPS.fix)); 
                   
   CAN0.sendMsgBuf(0x43e, 0, 8,  data );
   
    
    byte * altBuffer = (byte *) &GPS.altitude;
//    Serial.println(lon,8);
//    Serial.println(int(lon));
    double fractions = (lon - int(lon))*10000000; 
    unsigned long fracLongBuff= long(fractions);
//    Serial.println(fracLongBuff);
//     Serial.println(fracLongBuff,HEX);
//      Serial.println(byte((fracLongBuff & 0x00FF0000) >> 16),HEX);
//     Serial.println(byte((fracLongBuff & 0x0000FF00) >> 8),HEX);
//     Serial.println(byte(fracLongBuff & 0x000000FF),HEX);
//   Serial.println();
//    
     data[0] = byte(lon);
     data[1] = byte((fracLongBuff & 0x00FF0000) >> 16); 
     data[2] = byte((fracLongBuff & 0x0000FF00) >> 8) ;
     data[3] = byte(fracLongBuff & 0x000000FF);
     
     fractions = (lat - int(lat))*10000000; 
     unsigned long fracLatBuff= long(fractions);
    
     data[4] = byte(lat); 
     data[5] = byte((fracLatBuff & 0x00FF0000) >> 16);
     data[6] = byte((fracLatBuff & 0x0000FF00) >> 8) ;
     data[7] = byte(fracLatBuff & 0x000000FF);
                   
     CAN0.sendMsgBuf(0x43f, 0, 8,  data );
  
     data[0] = byte(GPS.day);
     data[1] = byte(GPS.month); 
     data[2] = highByte(int(GPS.year)); 
     data[3] = lowByte(int(GPS.year)); 
     data[4] = altBuffer[0]; 
     data[5] = altBuffer[1]; 
     data[6] = altBuffer[2]; 
     data[7] = altBuffer[3]; 
                   
    CAN0.sendMsgBuf(0x440, 0, 8,  data );
  
    Serial.print(currentMillis);
    Serial.print("\t");
    Serial.print(int (headingValue / 10));     // The whole number part of the heading
    Serial.print(".");
    Serial.print(abs(int (headingValue % 10)));     // The fractional part of the heading
    Serial.print("\t");
    Serial.print(GPS.speed);
    Serial.print("\t");
    Serial.print(GPS.angle);
    Serial.print("\t");
    Serial.print(GPS.satellites);
    Serial.print("\t");
    Serial.print(GPS.fix);
    Serial.print("\t");
    Serial.print(GPS.fixquality);
    Serial.print("\t");
    Serial.print(GPS.longitude,8);
    Serial.print("\t");
    Serial.print(lon,8);     // The whole number part of the heading
    Serial.print("\t");
    Serial.print(GPS.latitude,8);
    Serial.print("\t");
    Serial.print(lat,8);
    Serial.print("\t");
    Serial.print(GPS.altitude);
    Serial.print("\t");
    Serial.print(GPS.month);
    Serial.print("/");
    Serial.print(GPS.day);
    Serial.print("/");
    Serial.print(GPS.year);
    Serial.print("\t");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.print(GPS.milliseconds);
    Serial.print("\t");
    Serial.print(x2lon,8);
    Serial.print("\t");
    Serial.print(x2lat,8);
    Serial.print("\t");
    float dist = distance(x2lon, x2lat);
    Serial.print(dist);
    Serial.print("\t");
    headerValue = dirToFixedPoint (x2lon, x2lat );
    Serial.print(int (headerValue / 10));     // The whole number part of the heading
    Serial.print(".");
    Serial.println(int (headerValue % 10));     // The fractional part of the heading
    
     data[0] = highByte(int(headerValue)); 
     data[1] = lowByte(int(headerValue)); 
     data[2] = highByte(int(dist*10)); 
     data[3] = lowByte(int(dist*10)); 
     data[4] = byte(GPS.hour); 
     data[5] = byte(GPS.minute); 
     data[6] = byte(GPS.seconds); 
     data[7] = byte(GPS.milliseconds/10); 
   
     CAN0.sendMsgBuf(0x441, 0, 8,  data );
                   

  }
 
}

 
// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
double convertDegMinToDecDeg (float degMin) {
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}

double  distance(double x2lon, double x2lat ){
  double flat1=lat;     // flat1 = our current latitude. lat is from the gps data. 
  double flon1=lon;  // flon1 = our current longitude. lon is from the fps data.
 
  //------------------------------------------ distance formula below. Calculates distance from current location to waypoint
  diflat=radians(x2lat-flat1);  //notice it must be done in radians
  flat1=radians(flat1);    //convert current latitude to radians
  x2lat=radians(x2lat);  //convert waypoint latitude to radians
  diflon=radians((x2lon)-(flon1));   //subtract and convert longitudes to radians
  double dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(x2lat);
  dist_calc2*=sin(diflon/2.0);                                       
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;
  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  dist_calc*=6371000.0; //Converting to meters
  return dist_calc;
  
}

int dirToFixedPoint (double x2lon, double x2lat ){
   double flat1=lat;     // flat1 = our current latitude. lat is from the gps data. 
   double flon1=lon;  // flon1 = our current longitude. lon is from the fps data.
   flat1=radians(flat1);    //convert current latitude to radians
   x2lat=radians(x2lat);  //convert waypoint latitude to radians
   diflon=radians((x2lon)-(flon1));   //subtract and convert longitudes to radians
   flon1 = radians(flon1);  //also must be done in radians
   x2lon = radians(x2lon);  //radians duh.
   double head = atan2(sin(x2lon-flon1)*cos(x2lat),cos(flat1)*sin(x2lat)-sin(flat1)*cos(x2lat)*cos(x2lon-flon1))+2*3.1415926535;
   head = head*1800/3.1415926535;  // convert from radians to tenths of degrees

  int header =int(head); //make it a integer now

  if(header<0) header += 3600;   //if the heading is negative then add 360 to make it positive
  if(header>3600) header -= 3600;   //if the heading is big then subtract 360 to make it positive
  return header;
}
 
