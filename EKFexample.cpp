/*
   The Teensy Troller
   A program to demonstrate extended kalman filters

   By Jeremy Daily

   Released under the beer license: If you like it, I'd like to share a beer with you if we ever meet.
*/
#include <TinyGPS++.h> // Used to read the data from the GPS.
#include  <i2c_t3.h> // the I2C library that replaces Wire.h for the Teensy 3.2
#include <SFE_HMC6343.h> //Sparkfun's library for the digital compass

double gyroScaleFactor = 0.003814697265625; //Set to 125deg/s / 2^15
double gyroOffset = -0.122513335; //Used to zero out the rate gyro when it is still. Uses a longterm average.
#include "BNO055.h"

SFE_HMC6343 compass; // Declare the sensor object

//Initialize the GPS
TinyGPSPlus gps;

elapsedMillis gyroReadingTimer;

//Set up the Kalman filter to smooth the data from the Compass and Rate Gyro.
// These must be defined before including TinyEKF.h
#define N 2     // three state values: yawAngle, yawRate
#define M 3     // Four measurements: GPS Heading, compassHeading, rateGyro, 

//Call the Extended Kalman filter to set up sensor fusion for the rate gyro and the compass.
#include <TinyEKF.h>

//declare this here so it can be used in the Fuser model.
double deltaT = 0.05;
double deltaTms = 50;

class Fuser : public TinyEKF {

  public:

    Fuser()
    {
      // We approximate the process noise using a small constant
      this->setQ(0, 0, .0001);
      this->setQ(1, 1, .0001);
      this->setQ(2, 2, .0001);
     
      // Same for measurement noise (MxM diagnonal matrix assumes no covariance)
      this->setR(0, 0, 0.63612499); //Varianance of the noise from the GPS Heading
      this->setR(1, 1, 20.37804499); //Variance of the noise from the Compass
      this->setR(2, 2, .1756); //rate gyro (deg/s)^2     
    }

  protected:

    void model(double fx[N], double F[N][N], double hx[M], double H[M][N])
    {
      // Process model is f(x) = x
      fx[0] = this->x[0] + this->x[1] * deltaT; //degrees 
      fx[1] = this->x[1]; // deg/s
      
      //Process model Jacobian
      F[0][0] = 1; //df[0]/dx[0]
      F[0][1] = deltaT; //df[0]/dx[1]
      F[1][0] = 0; //df[1]/dx[0]
      F[1][1] = 1; //df[1]/dx[1]

      // Measurement function
      hx[0] = this->x[0]; // Yaw Angle from previous state
      hx[1] = this->x[0]; // yaw Angle from previous state
      hx[2] = this->x[1]; // yaw Angle from previous state
      
      // Jacobian of measurement function
      H[0][0] = 1;       // Yaw Angle from previous state
      H[1][1] = 1;       // yaw Rate from previous state
      H[2][2] = 1; 
    }
};

Fuser ekf;
double ekfYawAngle;
double ekfYawRate;
double compassHeading = 0.0;

void getMeasurements() {
  //measure stuff
  if (gyroReadingTimer >= deltaTms) {
    gyroReadingTimer = 0;
    yawRate = BNOgetYawRate();
    compassHeading = getCompassHeading();
    // Send these measurements to the EKF
    double z[M] = {gps.course.deg(), compassHeading, yawRate};
    ekfYawAngle = ekf.getX(0);
    ekfYawRate = ekf.getX(1);
    if (ekfYawAngle >= 360) ekfYawAngle -= 360;
    if (ekfYawAngle < 0)    ekfYawAngle += 360;
    ekf.setX(0, ekfYawAngle);
  }
  while (Serial1.available())
  {
    char c = Serial1.read();
    gps.encode(c);
  }
}

double getCompassHeading() {
  compass.readHeading();
  double compassOffset = 35.0; //degrees (This needs to be changed)
  double tempHeading = compass.heading / 10.0 + compassOffset; //local compass
  if (ekfYawAngle > 270 && tempHeading < 90) tempHeading += 360;
  if (ekfYawAngle < 90 && tempHeading > 270) tempHeading -= 360;
  return tempHeading;
}


void setup() {
  Wire.begin();
  Serial.begin(115200); //debug console
  BNOwrite(BNO055_OPR_MODE_ADDR, OPERATION_MODE_AMG); //
  delay(10);
  BNOwrite(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  delay(10);
  Serial.println("Done.");
    
  Serial.print("Starting GPS... ");
  //tft.println("Starting GPS");
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
}
    
void loop() {
  getMeasurements();
}