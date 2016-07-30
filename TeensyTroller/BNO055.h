#include  <i2c_t3.h>

const double AccelXOffset = 0.298012168 ;

//Bosch Sensortec BNO055 Definitions
//The following definitions are from the BNO055 Datasheet
#define BNO055_ADDRESS                                   0x28
#define BNO055_ADDRESS_A                                 0x28
#define BNO055_ADDRESS_B                                 0x29
#define BNO055_ID                                        0xA0
#define BNO055_PAGE_ID_ADDR                              0x07

/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                              0x00
#define BNO055_ACCEL_REV_ID_ADDR                         0x01
#define BNO055_MAG_REV_ID_ADDR                           0x02
#define BNO055_GYRO_REV_ID_ADDR                          0x03
#define BNO055_SW_REV_ID_LSB_ADDR                        0x04
#define BNO055_SW_REV_ID_MSB_ADDR                        0x05
#define BNO055_BL_REV_ID_ADDR                            0x06

/* Accel data register */
#define BNO055_ACCEL_DATA_X_LSB_ADDR                     0x08
#define BNO055_ACCEL_DATA_X_MSB_ADDR                     0x09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR                     0x0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR                     0x0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR                     0x0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR                     0x0D
/* Mag data register */
#define BNO055_MAG_DATA_X_LSB_ADDR                       0x0E
#define BNO055_MAG_DATA_X_MSB_ADDR                       0x0F
#define BNO055_MAG_DATA_Y_LSB_ADDR                       0x10
#define BNO055_MAG_DATA_Y_MSB_ADDR                       0x11
#define BNO055_MAG_DATA_Z_LSB_ADDR                       0x12
#define BNO055_MAG_DATA_Z_MSB_ADDR                       0x13

/* Gyro data registers */
#define BNO055_GYRO_DATA_X_LSB_ADDR                      0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR                      0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR                      0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR                      0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR                      0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR                      0x19

/* Euler data registers */
#define BNO055_EULER_H_LSB_ADDR                          0x1A
#define BNO055_EULER_H_MSB_ADDR                          0x1B
#define BNO055_EULER_R_LSB_ADDR                          0x1C
#define BNO055_EULER_R_MSB_ADDR                          0x1D
#define BNO055_EULER_P_LSB_ADDR                          0x1E
#define BNO055_EULER_P_MSB_ADDR                          0x1F

/* Quaternion data registers */
#define BNO055_QUATERNION_DATA_W_LSB_ADDR                0x20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR                0x21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR                0x22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR                0x23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR                0x24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR                0x25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR                0x26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR                0x27

/* Linear acceleration data registers */
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR              0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR              0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR              0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR              0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR              0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR              0x2D
 
/* Gravity data registers */
#define BNO055_GRAVITY_DATA_X_LSB_ADDR                   0x2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR                   0x2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR                   0x30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR                   0x31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR                   0x32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR                   0x33

/* Temperature data register */
#define BNO055_TEMP_ADDR                                 0x34

/* Status registers */
#define BNO055_CALIB_STAT_ADDR                           0x35
#define BNO055_SELFTEST_RESULT_ADDR                      0x36
#define BNO055_INTR_STAT_ADDR                            0x37

#define BNO055_SYS_CLK_STAT_ADDR                         0x38
#define BNO055_SYS_STAT_ADDR                             0x39
#define BNO055_SYS_ERR_ADDR                              0x3A

/* Unit selection register */
#define BNO055_UNIT_SEL_ADDR                             0x3B
#define BNO055_DATA_SELECT_ADDR                          0x3C

/* Mode registers */
#define BNO055_OPR_MODE_ADDR                             0x3D
#define BNO055_PWR_MODE_ADDR                             0x3E
#define BNO055_SYS_TRIGGER_ADDR                          0x3F
#define BNO055_TEMP_SOURCE_ADDR                          0x40

/* Axis remap registers */
#define BNO055_AXIS_MAP_CONFIG_ADDR                      0x41
#define BNO055_AXIS_MAP_SIGN_ADDR                        0x42

/* SIC registers */
#define BNO055_SIC_MATRIX_0_LSB_ADDR                     0x43
#define BNO055_SIC_MATRIX_0_MSB_ADDR                     0x44
#define BNO055_SIC_MATRIX_1_LSB_ADDR                     0x45
#define BNO055_SIC_MATRIX_1_MSB_ADDR                     0x46
#define BNO055_SIC_MATRIX_2_LSB_ADDR                     0x47
#define BNO055_SIC_MATRIX_2_MSB_ADDR                     0x48
#define BNO055_SIC_MATRIX_3_LSB_ADDR                     0x49
#define BNO055_SIC_MATRIX_3_MSB_ADDR                     0x4A
#define BNO055_SIC_MATRIX_4_LSB_ADDR                     0x4B
#define BNO055_SIC_MATRIX_4_MSB_ADDR                     0x4C
#define BNO055_SIC_MATRIX_5_LSB_ADDR                     0x4D
#define BNO055_SIC_MATRIX_5_MSB_ADDR                     0x4E
#define BNO055_SIC_MATRIX_6_LSB_ADDR                     0x4F
#define BNO055_SIC_MATRIX_6_MSB_ADDR                     0x50
#define BNO055_SIC_MATRIX_7_LSB_ADDR                     0x51
#define BNO055_SIC_MATRIX_7_MSB_ADDR                     0x52
#define BNO055_SIC_MATRIX_8_LSB_ADDR                     0x53
#define BNO055_SIC_MATRIX_8_MSB_ADDR                     0x54

/* Accelerometer Offset registers */
#define ACCEL_OFFSET_X_LSB_ADDR                          0x55
#define ACCEL_OFFSET_X_MSB_ADDR                          0x56
#define ACCEL_OFFSET_Y_LSB_ADDR                          0x57
#define ACCEL_OFFSET_Y_MSB_ADDR                          0x58
#define ACCEL_OFFSET_Z_LSB_ADDR                          0x59
#define ACCEL_OFFSET_Z_MSB_ADDR                          0x5A

/* Magnetometer Offset registers */
#define MAG_OFFSET_X_LSB_ADDR                            0x5B
#define MAG_OFFSET_X_MSB_ADDR                            0x5C
#define MAG_OFFSET_Y_LSB_ADDR                            0x5D
#define MAG_OFFSET_Y_MSB_ADDR                            0x5E
#define MAG_OFFSET_Z_LSB_ADDR                            0x5F
#define MAG_OFFSET_Z_MSB_ADDR                            0x60

/* Gyroscope Offset register s*/
#define GYRO_OFFSET_X_LSB_ADDR                           0x61
#define GYRO_OFFSET_X_MSB_ADDR                           0x62
#define GYRO_OFFSET_Y_LSB_ADDR                           0x63
#define GYRO_OFFSET_Y_MSB_ADDR                           0x64
#define GYRO_OFFSET_Z_LSB_ADDR                           0x65
#define GYRO_OFFSET_Z_MSB_ADDR                           0x66

/* Radius registers */
#define ACCEL_RADIUS_LSB_ADDR                            0x67
#define ACCEL_RADIUS_MSB_ADDR                            0x68
#define MAG_RADIUS_LSB_ADDR                              0x69
#define MAG_RADIUS_MSB_ADDR                              0x6A

    
#define POWER_MODE_NORMAL                                0x00
#define POWER_MODE_LOWPOWER                              0x01
#define POWER_MODE_SUSPEND                               0x02
#define GYRO_CONF_1                                      0x0B
#define GYRO_CONF_0                                      0x0A
#define MAG_CONF                                         0x09
#define ACC_CONF                                         0x08
      
 /* Operation mode settings*/
#define OPERATION_MODE_CONFIG                            0x00
#define OPERATION_MODE_ACCONLY                           0x01
#define OPERATION_MODE_MAGONLY                           0x02
#define OPERATION_MODE_GYRONLY                           0x03
#define OPERATION_MODE_ACCMAG                            0x04
#define OPERATION_MODE_ACCGYRO                           0x05
#define OPERATION_MODE_MAGGYRO                           0x06
#define OPERATION_MODE_AMG                               0x07
#define OPERATION_MODE_IMUPLUS                           0x08
#define OPERATION_MODE_COMPASS                           0x09
#define OPERATION_MODE_M4G                               0x0A
#define OPERATION_MODE_NDOF_FMC_OFF                      0x0B
#define OPERATION_MODE_NDOF                              0x0C

#define REMAP_CONFIG_P0                                  0x21
#define REMAP_CONFIG_P1                                  0x24 // default
#define REMAP_CONFIG_P2                                  0x24
#define REMAP_CONFIG_P3                                  0x21
#define REMAP_CONFIG_P4                                  0x24
#define REMAP_CONFIG_P5                                  0x21
#define REMAP_CONFIG_P6                                  0x21
#define REMAP_CONFIG_P7                                  0x24
#define REMAP_SIGN_P0                                    0x04
#define REMAP_SIGN_P1                                    0x00 // default
#define REMAP_SIGN_P2                                    0x06
#define REMAP_SIGN_P3                                    0x02
#define REMAP_SIGN_P4                                    0x03
#define REMAP_SIGN_P5                                    0x01
#define REMAP_SIGN_P6                                    0x07
#define REMAP_SIGN_P7                                    0x05

double yawRate = 0.0;
double yawAngle = 0.0;
 
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

double BNOgetYawRate(){
  byte gyroMSB = BNOread(BNO055_GYRO_DATA_Z_MSB_ADDR);
  byte gyroLSB = BNOread(BNO055_GYRO_DATA_Z_LSB_ADDR);
  int16_t tempGyro = word(gyroMSB,gyroLSB);
  return tempGyro * gyroScaleFactor + gyroOffset; //This is from 125 deg/second range 125/2^15 and subtracting off an average
  //Serial.println(yawRate,8);
}

//void BNOgetYawAngle(){
//  byte eulerMSB = BNOread(BNO055_EULER_H_MSB_ADDR);
//  byte eulerLSB = BNOread(BNO055_EULER_H_LSB_ADDR);
//  int16_t tempEuler = word(eulerMSB,eulerLSB);
//  yawAngle = tempEuler * headingScaleFactor + headingOffset; 
//  //Serial.println(yawRate,8);
//}

double BNOgetAccelX(){
  byte accelMSB = BNOread(BNO055_ACCEL_DATA_Y_MSB_ADDR ); //Change these to match the compass in the teensy troller
  byte accelLSB = BNOread(BNO055_ACCEL_DATA_Y_LSB_ADDR );
  int16_t tempAccel= word(accelMSB,accelLSB);
  return  double(tempAccel)/1000.0 - AccelXOffset  ; //; //in milligs Change this offset
  
}

uint8_t getBNO055Status(void)
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
    return gyro_config;
}
