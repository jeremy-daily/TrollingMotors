#include <SFE_HMC6343.h>

#include <TinyGPS.h>

#include <FlexCAN.h>
#include <kinetis_flexcan.h>

#include "SPI.h"
#include "ILI9341_t3.h"
#include  <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h> 

Adafruit_BNO055 bno = Adafruit_BNO055();
SFE_HMC6343 compass; // Declare the sensor object

IntervalTimer broadcastCANtimer;
elapsedMillis waitingForCAN;

elapsedMillis printTFTtimer;

int mode = 0; 
int numberOfModes = 6; //This limits the number of displayed modes.
char modeNames[6][6]={"Off ","Man. ","Turn ","Fix ","Fig8 ", "Full"};

boolean rightButtonState = LOW;
boolean leftButtonState = LOW;
boolean downButtonState = LOW;
boolean pushButtonState = LOW;
boolean upButtonState = LOW;

TinyGPS gps;

long lat, lon;
unsigned long fix_age, time, date, speed, course;
unsigned long chars;
unsigned short sentences, failed_checksum;

// For the Adafruit shield, these are the default.
#define TFT_DC 20
#define TFT_CS 21

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

Servo rightServo;  // create servo object to control a servo 
Servo leftServo;  // create servo object to control a servo 

int rightVal;    // variable to read the value from the analog pin 
int leftVal;    // variable to read the value from the analog pin 

FlexCAN CANbus(500000);
static CAN_message_t txmsg,rxmsg;
uint32_t CANTXcount = 0;
uint32_t CANRXcount = 0;

uint32_t ID = 0;

imu::Vector<3> euler;
imu::Vector<3> gyro;

void setup() {
  delay(100);
  Serial.begin(115200);
  delay(100);
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(3);
  tft.println("Setting Up...");
  
  tft.print("Starting Srvo");
  rightServo.attach(23);  // attaches the servo on pin 23 to the servo object 
  leftServo.attach(16);  // attaches the servo on pin 16 to the servo object 
  
  tft.println("Starting CAN");
  delay(100);
  CANbus.begin();
  broadcastCANtimer.begin(sendCANmessages, 100000); //call the sendCANmessages every 0.100 seconds
  tft.println("Starting GPS");
  Serial1.begin(9600);
  tft.println("Starting IMU");
   /* Initialise the sensor */
  bno.begin();
  bno.setExtCrystalUse(true);
  
  tft.print("Starting Comp");
  compass.init();
  delay(1000);
  
  tft.fillScreen(ILI9341_BLACK);
} 

void sendCANmessages(){
  //GPS Messages
  gps.get_position(&lat, &lon, &fix_age);
  gps.stats(&chars, &sentences, &failed_checksum);

  if (fix_age == TinyGPS::GPS_INVALID_AGE)
    Serial.println("No fix detected");
  else if (fix_age > 5000)
    Serial.println("Warning: possible stale data!");
  else {
    txmsg.id=0x43e;
    txmsg.len=8;
    txmsg.buf[0]=highByte(sentences);
    txmsg.buf[1]=lowByte(sentences);
    txmsg.buf[2]=highByte(gps.speed());
    txmsg.buf[3]=lowByte(gps.speed());
    txmsg.buf[4]=highByte(gps.course());
    txmsg.buf[5]=lowByte(gps.course());
    txmsg.buf[6]=byte(gps.satellites());
    txmsg.buf[7]=byte(fix_age);
    CANbus.write(txmsg);
    CANTXcount++;
  }
}
  


void readCANmessages(){
  while ( CANbus.read(rxmsg) ) {
      waitingForCAN = 0; //reset the can message timeout
      CANRXcount++;
      ID = rxmsg.id;
      if (ID == 0x777)
      {
        mode=rxmsg.buf[0];
        upButtonState=bitRead(rxmsg.buf[1],0);
        downButtonState=bitRead(rxmsg.buf[1],1);
        leftButtonState=bitRead(rxmsg.buf[1],2);
        rightButtonState=bitRead(rxmsg.buf[1],3);
        pushButtonState=bitRead(rxmsg.buf[1],4);
//        modeEnable=bitRead(rxmsg.buf[1],7);
      }
  }
}

void displayData(){
  if (printTFTtimer > 50){
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
    float yawIMU = euler.x();
    tft.print(yawIMU);
    
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
    float yawRate = gyro.z();
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
      tft.print(gps.satellites());
    }
  }
}

 
void loop() {

  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  compass.readHeading();
  displayData();
  
  rightVal = 512;            // reads the value of the potentiometer (value between 0 and 1023) 
  rightVal = map(rightVal, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  rightServo.write(90);                  // sets the servo position according to the scaled value 


  leftVal = 512;            // reads the value of the potentiometer (value between 0 and 1023) 
  leftVal = map(leftVal, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  leftServo.write(90);                  // sets the servo position according to the scaled value 
  delay(15);
  readCANmessages();
  while (Serial1.available())
    gps.encode(Serial1.read());
 
    
}


