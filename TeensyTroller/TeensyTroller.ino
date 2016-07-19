#include <TinyGPS++.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include "SPI.h"
#include "ILI9341_t3.h"
#include "BNO055.h"
#include  <i2c_t3.h>
#include <SFE_HMC6343.h>
#include <Servo.h>
#include <EEPROM.h>

// These must be defined before including TinyEKF.h
#define N 2     // Two state values: yawAngle, yawRate
#define M 2     // Three measurements: compassHeading, rateGyro,

#include <TinyEKF.h>
double deltaT = 0;


class Fuser : public TinyEKF {

    public:

        Fuser()
        {            
            // We approximate the process noise using a small constant
            this->setQ(0, 0, .0001);
            this->setQ(1, 1, .0001);

            // Same for measurement noise (MxM diagnonal matrix assumes no covariance)
            this->setR(0, 0, 1.5);
            this->setR(1, 1, .002);
            
        }

    protected:

        void model(double fx[N], double F[N][N], double hx[M], double H[M][N])
        {
            // Process model is f(x) = x
            fx[0] = this->x[0] - this->x[1]*deltaT;
            fx[1] = this->x[1];
            

            // So process model Jacobian is identity matrix
            F[0][0] = 1; //df[0]/dx[0]
            F[0][1] = deltaT; //df[0]/dx[1]
            F[1][0] = 0; //df[1]/dx[0]
            F[1][1] = 1; //df[1]/dx[1]

            // Measurement function
            hx[0] = this->x[0]; // Yaw Angle from previous state
            hx[1] = this->x[1]; // yaw Rate from previous state
           
            // Jacobian of measurement function
            H[0][0] = 1;       // Yaw Angle from previous state
            H[1][1] = 1;       // yaw Rate from previous state
           
        }
};

Fuser ekf;




//PID Gain Constants. Tune these for best results.
double angleK = 2;
double angleI = .1;
double angleD = 1;

double speedK = .1;
double speedI = .01;
double speedD = 0;

double ekfYawAngle;
double ekfYawRate;

double gyroScaleFactor = 0.003814697265625; //Set to 125deg/s / 2^15
double gyroOffset = -0.122513335; //Used to zero out the rate gyro when it is still. Uses a longterm average.

double headingScaleFactor = 1; //
double headingOffset = 0; //


double biasSetting = 1.0; // adjust this value to make the boat go straight in full mode. This the compensation coefficent for the right motor

const int memorySize = 1200.;
int32_t differenceList[1200];
int32_t differenceSpeedList[1200];

double yawAngleRaw; 

const uint32_t deltaTms = 50; //milliseconds for calculations and output

double yawOffset = 0;
double compassOffset = 0;
double CANcompassOffset = 0;
double initialYaw;
boolean isCompassOffsetSet = false;

double turnRate = 0.01;
double CANheading;
double CANcompassHeading;
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
elapsedMillis anchorAdjustTimer;

const int speedSetTime = 100; //set how quickly the speed changes.
const int courseSetTime = 100; //set how quickly the speed changes.

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



//Set up CAN messaging
FlexCAN CANbus(500000);
static CAN_message_t txmsg, rxmsg;
uint32_t CANTXcount = 0;
uint32_t CANRXcount = 0;
uint32_t ID = 0;
char message[17] = "                "; //initialize with spaces

// setup the IMU sensor
//Adafruit_BNO055 bno = Adafruit_BNO055();
//imu::Vector<3> euler;
//imu::Vector<3> gyro;

float yawAngle = 0.0;
float yawRate = 0.0;
float compassHeading = 0.0;


byte BNOread(int reg){
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(uint8_t(reg));
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom(BNO055_ADDRESS, 1);
  return Wire.read();
}

void BNOreadN(int reg, byte *dataBuffer, int len){
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(uint8_t(reg));
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom(BNO055_ADDRESS, len);
  for (uint8_t i = 0; i < len; i++) dataBuffer[i] = Wire.read();
}

void BNOwrite(int reg, uint8_t val){
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(uint8_t(reg));
  Wire.write(uint8_t(val));
  Wire.endTransmission();
}

void getBNO055Status(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    char statusMessage[40];
    
    BNOwrite(BNO055_PAGE_ID_ADDR,0); // Set to page 0

    uint8_t clock_select = BNOread(BNO055_SYS_TRIGGER_ADDR);
    sprintf(statusMessage,"Clock Select Bit: 0x%02X ",clock_select);
    Serial.print(statusMessage);
    if ((clock_select & 0x80) >> 7) Serial.println(" External Clock Selected");
    else Serial.println(" There seems to be a problem with the external clock.");

    uint8_t operation_mode = BNOread(BNO055_OPR_MODE_ADDR);
    sprintf(statusMessage,"Operation Mode: 0x%02X ",operation_mode);
    Serial.print(statusMessage);
    if      (operation_mode == 0) Serial.println(" CONFIG Mode");
    else if (operation_mode == 1) Serial.println(" ACCEL ONLY Mode");
    else if (operation_mode == 2) Serial.println(" MAG ONLY Mode");
    else if (operation_mode == 3) Serial.println(" GYRO ONLY Mode");
    else if (operation_mode == 4) Serial.println(" ACCMAG ONLY Mode");
    else if (operation_mode == 5) Serial.println(" ACCGYRO ONLY Mode");
    else if (operation_mode == 6) Serial.println(" MAGGYRO ONLY Mode");
    else if (operation_mode == 7) Serial.println(" AMG Mode");
    else if (operation_mode == 8) Serial.println(" IMU Fusion Mode");
    else if (operation_mode == 9) Serial.println(" COMPASS Fusion Mode");
    else if (operation_mode == 10) Serial.println(" M4G Fusion Mode");
    else if (operation_mode == 11) Serial.println(" NDOF_FMC_OFF Fusion Mode");
    else if (operation_mode == 12) Serial.println(" NDOF Fusion Mode");
    else Serial.println(" There seems to be a problem with the Operation Mode");

     
    uint8_t self_test_result = BNOread(BNO055_SELFTEST_RESULT_ADDR);
    sprintf(statusMessage,"Self Test Result: 0x%02X ",self_test_result);
    Serial.print(statusMessage);
    if (self_test_result == 0xf) Serial.println(" All systems passed the Self Test");
    else Serial.println(" There seems to be a problem.");

    uint8_t system_status    = BNOread(BNO055_SYS_STAT_ADDR);
    sprintf(statusMessage,"System Status: 0x%02X ",system_status);
    Serial.print(statusMessage);
    if      (system_status == 0) Serial.println("System Idle");
    else if (system_status == 1) Serial.println("System Error");
    else if (system_status == 2) Serial.println("Initilaising peripherals");
    else if (system_status == 3) Serial.println("System Initialization");
    else if (system_status == 4) Serial.println("Executing Self Test");
    else if (system_status == 5) Serial.println("Sensor Fusion Algorithm Running");
    else if (system_status == 6) Serial.println("System running without fusion algorithm");
    else Serial.println(" There seems to be a problem.");
         
    
    if (system_status == 1){
      uint8_t system_error     = BNOread(BNO055_SYS_ERR_ADDR);
      sprintf(statusMessage,"System Error: 0x%02X ",system_error);
      Serial.print(statusMessage);
      if (system_error == 0) Serial.println(" All systems passed the Self Test");
      else Serial.println(" There seems to be a problem with system status.");
    }

    uint8_t unit_selected    = BNOread(BNO055_UNIT_SEL_ADDR);
    sprintf(statusMessage,"Unit Select: 0x%02X ",unit_selected);
    Serial.println(statusMessage);
    if (unit_selected & 1 == 0) Serial.println("Accelerations are in m/s/s.");
    else Serial.println("Accelerations are in milli g.");
    if ((unit_selected & 2) >> 1 == 0) Serial.println("Rate Gyro is in Degrees/Second.");
    else Serial.println("Rate Gyro in Radians/Second.");
    if ((unit_selected & 4) >> 2 == 0) Serial.println("Euler Angle is in Degrees.");
    else Serial.println("Euler Angle is in Radians.");
    if ((unit_selected & 0x10) >> 4 == 0) Serial.println("Temperature is in Celcius.");
    else Serial.println("Temperature is in Farenheit.");

    BNOwrite(BNO055_PAGE_ID_ADDR,1); // Set to page 1
    
    uint8_t gyro_config    = BNOread(GYRO_CONF_0);
    sprintf(statusMessage,"Gyro Configuration 0: 0x%02X ",gyro_config);
    Serial.println(statusMessage);
    if      ((gyro_config & 0b0111) == 0) Serial.println("Gyro Range is 2000 dps.");
    else if ((gyro_config & 0b0111) == 1) Serial.println("Gyro Range is 1000 dps.");
    else if ((gyro_config & 0b0111) == 2) Serial.println("Gyro Range is 500 dps.");
    else if ((gyro_config & 0b0111) == 3) Serial.println("Gyro Range is 250 dps.");
    else if ((gyro_config & 0b0111) == 4) Serial.println("Gyro Range is 125 dps.");
    else Serial.println("There is something wrong with the Gyro Range reading.");
    
    if      ((gyro_config & 0b0111000)>>3 == 0) Serial.println("Gyro Bandwidth is 523 Hz.");
    else if ((gyro_config & 0b0111000)>>3 == 1) Serial.println("Gyro Bandwidth is 230 Hz.");
    else if ((gyro_config & 0b0111000)>>3 == 2) Serial.println("Gyro Bandwidth is 116 Hz.");
    else if ((gyro_config & 0b0111000)>>3 == 3) Serial.println("Gyro Bandwidth is 47 Hz.");
    else if ((gyro_config & 0b0111000)>>3 == 4) Serial.println("Gyro Bandwidth is 23 Hz.");
    else if ((gyro_config & 0b0111000)>>3 == 5) Serial.println("Gyro Bandwidth is 12 Hz.");
    else if ((gyro_config & 0b0111000)>>3 == 6) Serial.println("Gyro Bandwidth is 64 Hz.");
    else if ((gyro_config & 0b0111000)>>3 == 7) Serial.println("Gyro Bandwidth is 32 Hz.");
    else Serial.println("There is something wrong with the Gyro Bandwidth reading.");


    uint8_t mag_config    = BNOread(MAG_CONF);
    sprintf(statusMessage,"Magentometer Configuration: 0x%02X ",mag_config);
    Serial.println(statusMessage);
    Serial.print("Magnetometer Output Data Rate is ");
    if      ((mag_config & 0b0111) == 0) Serial.println("2 Hz.");
    else if ((mag_config & 0b0111) == 1) Serial.println("6 Hz.");
    else if ((mag_config & 0b0111) == 2) Serial.println("8 Hz.");
    else if ((mag_config & 0b0111) == 3) Serial.println("10 Hz.");
    else if ((mag_config & 0b0111) == 4) Serial.println("15 Hz.");
    else if ((mag_config & 0b0111) == 5) Serial.println("20 Hz.");
    else if ((mag_config & 0b0111) == 6) Serial.println("25 Hz.");
    else if ((mag_config & 0b0111) == 7) Serial.println("30 Hz.");
    else Serial.println("XX. There is something wrong with the Magnetometer data output reading.");

    Serial.print("Magnetometer Operation Mode is ");
    if      ((mag_config & 0b011000)>>3 == 0) Serial.println("Low Power");
    else if ((mag_config & 0b011000)>>3 == 1) Serial.println("Regular");
    else if ((mag_config & 0b011000)>>3 == 2) Serial.println("Enhanced Regular");
    else if ((mag_config & 0b011000)>>3 == 3) Serial.println("High Accuracy");
    else Serial.println("XX. There is something wrong with the Magnetometer data output reading.");
    
    uint8_t accel_config    = BNOread(ACC_CONF);
    sprintf(statusMessage,"Accelerometer Configuration: 0x%02X ",accel_config);
    Serial.println(statusMessage);
    Serial.print("Accelerometer G Range is ");
    if      ((accel_config & 0b011) == 0) Serial.println("2 G.");
    else if ((accel_config & 0b011) == 1) Serial.println("4 G.");
    else if ((accel_config & 0b011) == 2) Serial.println("8 G.");
    else if ((accel_config & 0b011) == 3) Serial.println("16 G.");
    else Serial.println("XX. There is something wrong with the Accelerometer G Range reading.");

    Serial.print("Accelerometer Bandwidth is ");
    if      ((accel_config & 0b011100)>>2 == 0) Serial.println("7.81 Hz.");
    else if ((accel_config & 0b011100)>>2 == 1) Serial.println("15.63 Hz");
    else if ((accel_config & 0b011100)>>2 == 2) Serial.println("31.25 Hz");
    else if ((accel_config & 0b011100)>>2 == 3) Serial.println("62.5 Hz");
    else if ((accel_config & 0b011100)>>2 == 4) Serial.println("125 Hz");
    else if ((accel_config & 0b011100)>>2 == 5) Serial.println("250 Hz");
    else if ((accel_config & 0b011100)>>2 == 6) Serial.println("500 Hz");
    else if ((accel_config & 0b011100)>>2 == 7) Serial.println("1000 Hz");
    else Serial.println("XX. There is something wrong with the Accelerometer Bandwidth reading.");
 
    BNOwrite(BNO055_PAGE_ID_ADDR,0); // Set back to page 0

    uint8_t calibraton_status    = BNOread(BNO055_CALIB_STAT_ADDR);
    sprintf(statusMessage,"Calibration Status: 0x%02X ",calibraton_status);
    Serial.println(statusMessage);
    Serial.print("System Calibration is  ");
    if      ((calibraton_status & 0b11000000)>>6 == 0) Serial.println("Not Calibrated.");
    else if ((calibraton_status & 0b11000000)>>6 == 3) Serial.println("Fully Calibrated.");
    else Serial.println("Something in the middle.");
    Serial.print("Gyro Calibration is  ");
    if      ((calibraton_status & 0b00110000)>>4 == 0) Serial.println("Not Calibrated.");
    else if ((calibraton_status & 0b00110000)>>4 == 3) Serial.println("Fully Calibrated.");
    else Serial.println("Something in the middle.");
    Serial.print("Accelerometer Calibration is  ");
    if      ((calibraton_status & 0b00001100)>>2 == 0) Serial.println("Not Calibrated.");
    else if ((calibraton_status & 0b00001100)>>2 == 3) Serial.println("Fully Calibrated.");
    else Serial.println("Something in the middle.");
    Serial.print("Magentometer Calibration is  ");
    if      ((calibraton_status & 0b00000011) == 0) Serial.println("Not Calibrated.");
    else if ((calibraton_status & 0b00000011) == 3) Serial.println("Fully Calibrated.");
    else Serial.println("Something in the middle.");
    
}

void BNOgetYawRate(){
  byte gyroMSB = BNOread(BNO055_GYRO_DATA_Z_MSB_ADDR);
  byte gyroLSB = BNOread(BNO055_GYRO_DATA_Z_LSB_ADDR);
  int16_t tempGyro = word(gyroMSB,gyroLSB);
  yawRate = tempGyro * gyroScaleFactor + gyroOffset; //This is from 125 deg/second range 125/2^15 and subtracting off an average
  //Serial.println(yawRate,8);
}

void BNOgetYawAngle(){
  byte eulerMSB = BNOread(BNO055_EULER_H_MSB_ADDR);
  byte eulerLSB = BNOread(BNO055_EULER_H_LSB_ADDR);
  int16_t tempEuler = word(eulerMSB,eulerLSB);
  yawAngle = tempEuler * headingScaleFactor + headingOffset; //This is from 125 deg/second range 125/2^15 and subtracting off an average
  //Serial.println(yawRate,8);
}


void setup() {

  Serial.begin(115200); //debug console
  delay(500);

  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(3);
  tft.print("Setting Up...");
  Serial.println("Welcome to the Teensy Troller. We will get setup first.");
  
  tft.println("Starting IMU");
  Serial.print("Starting Bosch BNO055 IMU Sensor... ");
  Wire.begin();
  BNOwrite(BNO055_OPR_MODE_ADDR,OPERATION_MODE_CONFIG); //Set to configuration Mode
  delay(10);
  Serial.println("Done.");
  
  // Set to use external oscillator
  Serial.print("Setting up for External oscillator... ");
  if(BNOread(BNO055_SYS_CLK_STAT_ADDR) == 0){
    BNOwrite(BNO055_SYS_TRIGGER_ADDR,0b10000000); 
    Serial.println("BNO055 set to use external oscillator");
  }
  else
  {
    BNOwrite(BNO055_SYS_TRIGGER_ADDR,0b10000000);
    Serial.println("Problem setting BNO055 to use external oscillator.");
  }
  delay(10);
  
  Serial.print("Setting up Rate Gyro... ");
  BNOwrite(BNO055_PAGE_ID_ADDR,1); // Set to page 1
  delay(10);
  BNOwrite(GYRO_CONF_0,0b00101100);  // Set gyroscope to 125deg/s at 23Hz (see pg 28 of datasheet)
  delay(10);
  BNOwrite(GYRO_CONF_1,0);  // Set gyroscope  (see pg 28 of datasheet)
  delay(10);
  Serial.println("Done.");

  Serial.print("Setting up Magnetometer... ");
  BNOwrite(MAG_CONF,0b00011001);  // 
  delay(10);
  Serial.println("Done.");

  Serial.print("Setting up Accelerometer... ");
  BNOwrite(ACC_CONF,0);  // 
  delay(10);
  Serial.println("Done.");

  Serial.print("Setting up Units... ");
  BNOwrite(BNO055_PAGE_ID_ADDR,0); // Set to page 0
  delay(10);
  BNOwrite(BNO055_UNIT_SEL_ADDR,0b00010000); // Set units to M/s^2 Deg/sec, Deg, Deg F
  delay(10);
  Serial.println("Done.");

  Serial.print("Turning on Sensors... ");
  BNOwrite(BNO055_OPR_MODE_ADDR,OPERATION_MODE_AMG);//
  //BNOwrite(BNO055_OPR_MODE_ADDR,OPERATION_MODE_NDOF);//
  Serial.println("Done.");

  Serial.println("Verifying BNO055 Settings...");
  getBNO055Status();
  Serial.println("Done.");
  
  

  Serial.print("Starting Servos... ");    
  
  tft.print("Starting Srvo");
  rightServo.attach(23);  // attaches the servo on pin 23 to the servo object
  leftServo.attach(16);  // attaches the servo on pin 16 to the servo object
  Serial.println("Done.");
 
  Serial.print("Starting GPS... ");
  tft.println("Starting GPS");
  Serial1.begin(9600);
  delay(300);
  Serial1.println("$PMTK251,57600*2C"); //Set Baud Rate to 57600
  delay(100);
  Serial1.flush();
  Serial1.end();
  Serial.println("Setting GPS to 57600 baud... ");
  Serial1.begin(57600);
  //delay(300);
  Serial1.println("$PMTK251,57600*2C"); //Set Baud Rate to 57600

  Serial.println("Setting GPS to update at 5 Hz... ");
  Serial1.println("$PMTK220,200*2C"); //update at 5 Hz
  delay(100);
  Serial1.println("$PMTK300,200,0,0,0,0*2F"); //position fix update to 5 Hz
  for (int i = 0;i<100;i++){
    if (Serial1.available()) Serial.write(Serial1.read());
  }
  Serial.println("\nDone.");
  

  Serial.println("Starting Compass... ");
  tft.print("Starting Comp");
  compass.init();
  byte bSN_LSB = compass.readEEPROM(SN_LSB);
  byte bSN_MSB = compass.readEEPROM(SN_MSB);
  tft.print("S/N:");
  tft.println(word(bSN_MSB, bSN_LSB));
  Serial.print("Compass Serial Number: ");
  Serial.println(word(bSN_MSB, bSN_LSB));
  compass.writeEEPROM(0x14, 0x00); //Set filter to 0.
  compass.exitStandby();
  delay(100);
  Serial.println("\nDone.");
  
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

  
  delay(1000);
  compassHeading = getCompassHeading();
  ekf.setX(0,compassHeading);
  deltaT = double(deltaTms) / 1000.0;

  displayTemplate(); //tft display
}

void resetCompassOffset() {
  if (gps.speed.mph() > 5.0 && abs(yawRate) < 3.0 ) //compass heading can be realigned to velocity vector
  {
    if (gps.course.deg() >= 270 && compass.heading / 10.0 <= 90)
    {
      compassOffset = gps.course.deg() - compass.heading / 10.0 + 360;
      CANcompassOffset = gps.course.deg() - CANheading + 360;
    }
    else if (gps.course.deg() <= 90 && compass.heading / 10.0 >= 270)
    {
      compassOffset = gps.course.deg() - compass.heading / 10.0 - 360;
      CANcompassOffset = gps.course.deg() - CANheading - 360;
    }
    else
    {
      compassOffset = gps.course.deg() - compass.heading / 10.0;
      CANcompassOffset = gps.course.deg() - CANheading;
    }
    Serial.print("Setting Compass Offsets Based on GPS heading: compassOffset = ");
    Serial.print(compassOffset);
    Serial.print(", CANcompassOffset = ");
    Serial.println(CANcompassOffset);
    isCompassOffsetSet = true;
    

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
  tft.print("CAN Angle:XXX");

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
    sprintf(dispVal, "%3i", int(CANcompassHeading));
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
  Serial.print("CANcompassHeading");
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
    Serial.print(CANcompassHeading);
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
  tempHeading = compass.heading/10.0 - compassOffset; //local compass
  CANcompassHeading = CANheading - CANcompassOffset;

  if (ekfYawAngle > 270 && tempHeading < 90) tempHeading += 360;
  if (ekfYawAngle < 90 && tempHeading > 270) tempHeading -= 360;
  
  return tempHeading;
}

double getYawAngle() {
  //euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  double tempyawAngle = yawAngleRaw - yawOffset;
  while (tempyawAngle >= 360) tempyawAngle -= 360;
  while (tempyawAngle < 0) tempyawAngle += 360;
  return tempyawAngle;
}

void getMeasurements() {
  //measure stuff
  if (gyroReadingTimer >= deltaTms) {
    gyroReadingTimer = 0;

    BNOgetYawRate();
    compassHeading = getCompassHeading();

    // Send these measurements to the EKF
    double z[M] = {compassHeading, yawRate};
    ekf.step(z);

    // Report measured and predicte/fused values
    Serial.print(z[0],4);
    Serial.print("\t");
    Serial.print(ekf.getX(0),4);
    Serial.print("\t");
    Serial.print(z[1],4);
    Serial.print("\t");
    Serial.println(ekf.getX(1),4);
    ekfYawAngle = ekf.getX(0);
    ekfYawRate = ekf.getX(1);
    if (ekfYawAngle >= 360) ekfYawAngle -= 360;
    if (ekfYawAngle < 0)    ekfYawAngle += 360;
    ekf.setX(0,ekfYawAngle);

/*Uncomment the lines below to get a stream of sensor data*/
    
//    Serial.print(compassHeading);
//    Serial.print("\t");
//    Serial.print(CANcompassHeading);
//    Serial.print("\t");
//    Serial.print(gps.speed.mph(),4);
//    Serial.print("\t");
//    Serial.println(yawRate,6);

  }

  //get user input
  readCANmessages();
  if (CANaliveTimer > 500) mode = 0;


  while (Serial1.available())
  {
    char c = Serial1.read();
    //Serial.print(c); //used for debugging
    gps.encode(c);
  }
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
      ////resetYawOffset()
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
      ////resetYawOffset()
    }
    

    displayMode3();
   if (mode3started) {
      if (anchorAdjustTimer > 300){
          anchorAdjustTimer = 0;
          if (upButtonState ) {
             fixPointLat +=  (10.0/111030.0)*sin(radians(compassHeading));
             fixPointLon += (10.0/111030.0)*cos(radians(compassHeading));
          }
          else if (downButtonState ) {
             fixPointLat -= (10.0/111030.0)*sin(radians(compassHeading));
             fixPointLon -= (10.0/80000.0)*cos(radians(compassHeading));
          }
          if (leftButtonState ) {
             fixPointLat += (10.0/111030.0)*cos(radians(compassHeading));
             fixPointLon += (10.0/80000.0)*sin(radians(compassHeading));
          }
          else if (rightButtonState ) {
             fixPointLat -= (10.0/111030.0)*cos(radians(compassHeading));
             fixPointLon -= (10.0/80000.0)*sin(radians(compassHeading));
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
    debugData();
    if (mode5started) {
      debugData();
      if (upButtonState) {
        rightMotor = int( 0.8*maxFwdMotorValue * biasSetting);
        leftMotor = int(0.8*maxFwdMotorValue);
      }
      else if (downButtonState) {
        rightMotor = int(0.8*maxRevMotorValue) * biasSetting;
        leftMotor = int(0.8*maxRevMotorValue);
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
    debugData();
    if (upButtonState && pushButtonState) {
      mode6started = true;
      compass.enterCalMode();
      delay(30);

      totalTurn = 0;
      lastAngle = compassHeading;
      yawOffset = yawAngleRaw;
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
    else if (upButtonState && !pushButtonState) {
      if (gps.location.isUpdated() && gps.speed.mph()>2){
      
        compass.readHeading();
        int  deviation = gps.course.deg() * 10 - compass.heading; // Not sure if this is the correct deviation method.
        compass.writeEEPROM(0x0A, lowByte(deviation)); //LSB of deviation
        delay(10);
        compass.writeEEPROM(0x0B, highByte(deviation)); //MSB of deviation
        delay(10);
      
        compass.reset();
        delayTimer = 0;
        while (delayTimer < 500) {
          while (Serial1.available()) gps.encode(Serial1.read());
          sendCANmessages();
        }
      }
    }
    else if (downButtonState && pushButtonState) {
      
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
        
        strncpy(message, "Forward ", 8);
        txmsg.id = 0x212; //sent to the lower right
        for (int j = 0; j < txmsg.len; j++) txmsg.buf[j] = message[j];
        CANbus.write(txmsg);
        
        resetCompassOffset();
        rightMotor = maxFwdMotorValue;
        leftMotor = maxFwdMotorValue;

        rightServo.write(rightMotor);
        leftServo.write(leftMotor);

        delayTimer = 0;
        while (delayTimer < 10000) {
          while (Serial1.available()) gps.encode(Serial1.read());
          sendCANmessages();
          if (mode != 7) {//abort
            resetOutputs();
            break;
          }
          displayMode6();
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
        delay(10);
        compass.writeEEPROM(0x0B, highByte(deviation)); //MSB of deviation
        delay(10);
        
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
        displayMode7();

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
  turnSetting = 0;
  angleSetting = 0;
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
    

    difference = goalAngle - ekfYawAngle; //using compass for input
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

    if (gps.location.isUpdated())
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

    angleSetting = int(angleK * difference + angleI * integral + angleD * ekfYawRate);


    int tempRightMotor = int((speedSetting - turnSetting - angleSetting + stopMotorValue)* biasSetting);
    int tempLeftMotor  = speedSetting + turnSetting + angleSetting + stopMotorValue ;
    rightMotor = constrain(tempRightMotor, maxRevMotorValue, maxFwdMotorValue);
    leftMotor  = constrain(tempLeftMotor, maxRevMotorValue, maxFwdMotorValue);


    //print
    debugData();
  }

}

