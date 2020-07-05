#include <Bounce2.h>
#include <FlexCAN.h>
#include <SerLCD.h> 

SerLCD lcd;

#define DEBOUNCE_TIME 20
#define CAN_TIMEOUT   1200
#define modeDisplayPeriod 85
#define MODE_CHANGE_DELAY 500

#define MOTOR_STATUS_ID       0x020
#define MOTOR_COMMAND_ID      0x010
#define JOYSTICK_STATUS_ID    0x700
#define GPS_STATUS_ID         0x43E

#define numModes 3
#define DEFAULT_MODE           0 
#define NORMAL_OPERATION_MODE  1
#define TURN_90_MODE           2

//Define the button input pins
#define rightButton 13
#define downButton  16
#define pushButton  12
#define leftButton  17
#define upButton    15
#define redButton   11
#define greenButton 10

#define STANDBY_PIN 7

Bounce rightDebouncer = Bounce(); 
Bounce downDebouncer  = Bounce(); 
Bounce pushDebouncer  = Bounce(); 
Bounce leftDebouncer  = Bounce(); 
Bounce upDebouncer    = Bounce(); 
Bounce redDebouncer   = Bounce(); 
Bounce greenDebouncer = Bounce(); 

bool rightButtonState;
bool downButtonState;
bool pushButtonState;
bool leftButtonState;
bool upButtonState;
bool redButtonState;
bool greenButtonState;

boolean writeOnce   = false;
boolean doubleClick = false;

elapsedMillis lastJoyTime;
elapsedMillis lastRXTime;
elapsedMillis command_timer;
elapsedMillis modeDisplayTimer;
elapsedMillis modeChangeTimer;
elapsedMillis calculateMotorOutputTimer;
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

uint32_t changeModeMillis;
uint8_t mode;

bool NORMAL_OPERATION_MODE_started = false;
bool TURN_90_MODE_started = false;

int8_t leftMotorCommand;
int8_t rightMotorCommand;

float gps_speed;
uint8_t gps_satellites;
float gps_course;

float turnRate = 0.85; //degrees per second
int turnTime = 100;
int changeLimit = 2;

const int stopMotorValue = 0;
const int maxRevMotorValue = -127;
const int maxFwdMotorValue = 127;

#define compassOffsetAddress 0
#define CANcompassOffsetAddress 8
double compassOffset = -10.8;

double angleK = 8;
double angleI = .3;
double angleD = 30;

//These still need tuned, but they seem to work.
double speedK = .1;
double speedI = .3;
double speedD = 0;

int feedforward = 10;
int feedforwardSpeed = 29;

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

byte currentMode = 0;
byte numberOfModes = 5; //This limits the number of displayed modes.
byte fig8phase = 0;


// Jackson Lake
double fixPointLat = 43.84825516;
double fixPointLon = -110.6231537;
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
int previousAngleSetting;
int speedAdjust;
int previousSpeedAdjust;

double compassHeading = 0.0;

boolean rightTurn = false;
boolean leftTurn = false;

float yawAngleRaw = 0;

uint8_t leftMotor = 92;
uint8_t rightMotor = 92;
uint8_t previousLeftMotor = 92;
uint8_t previousRightMotor = 92;
uint8_t motorChange = 1;

void setup() {
  Serial1.begin(9600);
  Serial.println("It's time to go fishing with the Dailys!!");
  Can0.begin(250000);
 
  // make the pushbutton's pin an input:
  pinMode(rightButton, INPUT_PULLUP);
  pinMode(leftButton,  INPUT_PULLUP);
  pinMode(downButton,  INPUT_PULLUP);
  pinMode(pushButton,  INPUT_PULLUP);
  pinMode(upButton,    INPUT_PULLUP);
  pinMode(redButton,   INPUT_PULLUP);
  pinMode(greenButton, INPUT_PULLUP);
  
  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN,LOW);
  
  rightDebouncer.attach(rightButton);
  leftDebouncer.attach(leftButton);
  downDebouncer.attach(downButton);
  pushDebouncer.attach(pushButton);
  upDebouncer.attach(upButton);
  redDebouncer.attach(redButton);
  greenDebouncer.attach(greenButton);
  
  rightDebouncer.interval(DEBOUNCE_TIME);
  leftDebouncer.interval(DEBOUNCE_TIME);
  downDebouncer.interval(DEBOUNCE_TIME);
  pushDebouncer.interval(DEBOUNCE_TIME);
  upDebouncer.interval(DEBOUNCE_TIME);
  redDebouncer.interval(DEBOUNCE_TIME);
  greenDebouncer.interval(DEBOUNCE_TIME); // interval in ms

  lcd.begin(Serial1); //Set up the LCD for Serial communication at 9600bps
  lcd.setBacklight(255, 255, 255); //Set backlight to bright white
  lcd.setContrast(7); //Set contrast. Lower to 0 for higher contrast.
  delay(500);
  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  delay(100);
  lcd.print("Let's go fishing");
  lcd.print("Fun for everyone");
  delay(2000);

  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  leftMotorCommand = 0;
  rightMotorCommand = 0;
}

// the loop routine runs over and over again forever:
void loop() { 
  rightDebouncer.update();
  leftDebouncer.update();
  downDebouncer.update();
  pushDebouncer.update();
  upDebouncer.update();
  redDebouncer.update();
  greenDebouncer.update();

  rightButtonState = rightDebouncer.read();
  leftButtonState = leftDebouncer.read();
  downButtonState = downDebouncer.read();
  pushButtonState = pushDebouncer.read();
  upButtonState = upDebouncer.read();
  redButtonState = redDebouncer.read();
  greenButtonState = greenDebouncer.read();

  lcd.setCursor(0, 0);
  if (!redButtonState){
    //lcd.print('r');
    Serial.println("Stop");
    lcd.setCursor(0, 0);
    lcd.print("Stop Btn Pushed ");
    leftMotorCommand = 0;
    rightMotorCommand = 0;
    mode = DEFAULT_MODE;
  }
  lcd.setCursor(15, 1);
  
  if (!greenButtonState){
    //lcd.print('g');
    if (modeChangeTimer > MODE_CHANGE_DELAY){
      if (!upButtonState) {
        modeChangeTimer = 0;
        mode +=1;
        if (mode > numberOfModes) mode = 0;
      }
      else if (!downButtonState) {
        modeChangeTimer = 0;
        mode -=1;
        if (mode < 0) mode = numberOfModes;
      }
    }
  }
  //Automatically move to the next slot and print Joystick positions.
//  if (!rightButtonState) lcd.print("R");
//  else if (!leftButtonState) lcd.print("L");
//  else if (!downButtonState) lcd.print("D");
//  else if (!upButtonState) lcd.print("U");
  
  readCANbus();
  state_machine();
  sendJoyStick();
  sendMotorCommand();
  display_LCD();
}

void state_machine(){
  
  //##############################################################################################
  //# Mode 1: Manual
  //##############################################################################################
  if (mode == NORMAL_OPERATION_MODE) { // Manual Mode
    if (NORMAL_OPERATION_MODE_started) {
      if (!rightButtonState && !leftButtonState) turnSetting = 0;
      if (speedSettingTimer > speedSetTime) {
        speedSettingTimer = 0;
        if (!upButtonState && pushButtonState) goalSpeed += 0.1; //mph
        if (!downButtonState && pushButtonState) goalSpeed -= 0.1;
        goalSpeed = constrain(goalSpeed, 0, 4);
        speedSetting = feedforwardSpeed * goalSpeed; //feed forward
      }
      if (courseSettingTimer > courseSetTime) {
        courseSettingTimer = 0;
        if (!leftButtonState && pushButtonState) {
          goalAngle -= 1;
          turnSetting = -60;
          leftTurn = false;
          rightTurn = false;
        }
        else if (!leftButtonState && !pushButtonState) {
          goalAngle = ekfYawAngle;
          turnTimer = 0;
          leftTurn = true;
          rightTurn = false;
        }
        else if (!rightButtonState && pushButtonState) {
          goalAngle += 1;
          turnSetting = 60;
          leftTurn = false;
          rightTurn = false;
        }
        else if (!rightButtonState && !pushButtonState) {
          goalAngle = ekfYawAngle;
          turnTimer = 0;
          leftTurn = false;
          rightTurn = true;
        }
      }
      if (turnTimer >= turnTime){
        turnTimer = 0;
      
        if (rightTurn ) { //right turn slowly 
          goalAngle += turnRate*turnTime/1000 ;
          turnSetting = feedforward; //feed forward
          
        }
        else if (leftTurn) { 
          goalAngle -= turnRate*turnTime/1000 ;
          turnSetting = -feedforward; //feed forward
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

    if (!upButtonState && !pushButtonState) {
      NORMAL_OPERATION_MODE_started = true;
      goalAngle = ekfYawAngle;
      memset(differenceList, 0, sizeof(differenceList)) ;
      leftTurn = false;
      rightTurn = false;
      turnSetting = 0;
      
    }
    else if (!downButtonState && !pushButtonState) {
      memset(differenceSpeedList, 0, sizeof(differenceSpeedList));
      goalSpeed = 0;
      goalAngle = ekfYawAngle;
      leftTurn = false;
      rightTurn = false;
    }

  }
  

}

/***********************************************************************************************/
/***********************************************************************************************/
void sendJoyStick(){
  //CAN interface messages
  CAN_message_t joyMessage;
  joyMessage.id = JOYSTICK_STATUS_ID;
  if (lastJoyTime >= 50){
    lastJoyTime = 0;
    bitWrite(joyMessage.buf[0],0,!upButtonState);
    bitWrite(joyMessage.buf[0],1,!downButtonState);
    bitWrite(joyMessage.buf[0],2,!leftButtonState);
    bitWrite(joyMessage.buf[0],3,!rightButtonState);
    bitWrite(joyMessage.buf[0],4,!pushButtonState);
    bitWrite(joyMessage.buf[0],5,doubleClick);
    bitWrite(joyMessage.buf[0],6,!redButtonState);
    bitWrite(joyMessage.buf[0],7,!greenButtonState);
    Can0.write(joyMessage);
//    char char_buffer [10];
//    itoa(joyMessage.buf[0],char_buffer,2);
//    Serial.printf("Joystick: %08s\n",char_buffer) 
  }
}
/***********************************************************************************************/
/***********************************************************************************************/

void sendMotorCommand(){
  CAN_message_t outMsg; 
  outMsg.id = MOTOR_COMMAND_ID;
  outMsg.len = 2;
  outMsg.buf[0] = leftMotorCommand;
  outMsg.buf[1] = rightMotorCommand;
  if (command_timer >=100){
    command_timer = 0;
    Can0.write(outMsg);
    Serial.printf("Command: %03X # %02X %02X\n",outMsg.id,outMsg.buf[0],outMsg.buf[1]); 
  }
}

/***********************************************************************************************/
/***********************************************************************************************/

void readCANbus(){
  CAN_message_t inMsg; 
  if (Can0.read(inMsg)){
    lastRXTime = 0;
    writeOnce=false;
  }
  
  if (lastRXTime > CAN_TIMEOUT){
    if (!writeOnce){
      lcd.setCursor(0, 1);
      lcd.print("Lost CAN Comms.");
      writeOnce=true;
    }
  }
}
/***********************************************************************************************/
/***********************************************************************************************/

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

    if (gps_speed)
    {
      speedDifference = goalSpeed - ekfSpeed;

      differenceSpeedList[diffIndex] = int32_t(speedDifference * 1000);
      diffSpeedIndex += 1;
      if (diffSpeedIndex >= memorySize) diffSpeedIndex = 0;

      int32_t speedSum = 0;
      for (int j = 0; j < memorySize; j++) speedSum += differenceSpeedList[j];

      double integralSpeed = double(speedSum) / 1000.0 * deltaT;

      speedAdjust = constrain(int(speedK * speedDifference + speedI * integralSpeed),-50,50);
      //Change the motor ouptut slowly to avoid pulsing motion.
      if (speedAdjust - previousSpeedAdjust > changeLimit){
        speedAdjust = previousSpeedAdjust + changeLimit;
      }
      else if (previousSpeedAdjust - speedAdjust > changeLimit){
        speedAdjust = previousSpeedAdjust - changeLimit;
      }
      previousSpeedAdjust = speedAdjust;
    }


    angleSetting = constrain(int(angleK * difference + angleI * integral + angleD * ekfYawRate),-100,100);
    //Change the motor ouptut slowly to avoid pulsing motion.
    if (angleSetting - previousAngleSetting > changeLimit){
      angleSetting = previousAngleSetting + changeLimit;
    }
    else if (previousAngleSetting - angleSetting > changeLimit){ // Switch order of operations for negative numbers
      angleSetting = previousAngleSetting - changeLimit;
    }
    previousAngleSetting = angleSetting;
    
    int tempRightMotor = int((speedSetting + speedAdjust - turnSetting - angleSetting + stopMotorValue) * biasSetting);
    int tempLeftMotor  = int( speedSetting + speedAdjust + turnSetting + angleSetting + stopMotorValue);
    
    if (tempRightMotor - previousRightMotor > motorChange){
      tempRightMotor = previousRightMotor + motorChange;
    }
    else if (previousRightMotor - tempRightMotor > motorChange){
      tempRightMotor = previousRightMotor - motorChange;
    }
    
    if (tempLeftMotor - previousLeftMotor > motorChange){
      tempLeftMotor = previousLeftMotor + motorChange;
    }
    else if (previousLeftMotor - tempLeftMotor > motorChange){
      tempLeftMotor = previousLeftMotor - motorChange;
    }
    
    rightMotor = constrain(tempRightMotor, maxRevMotorValue, maxFwdMotorValue);
    leftMotor  = constrain(tempLeftMotor,  maxRevMotorValue, maxFwdMotorValue);

    previousRightMotor = rightMotor;
    previousLeftMotor = leftMotor;
    
    rightMotorCommand = rightMotor;
    leftMotorCommand = leftMotor;
    
    //print
    //debugData();
  }
}

/***************************************************************************************/
/*Display Modes*************************************************************************/

void display_LCD() {
  if (modeDisplayTimer >= modeDisplayPeriod) {
    modeDisplayTimer = 0;
    if (mode == DEFAULT_MODE){
      lcd.setCursor(0,0);
      lcd.printf("OFF O:%5.1f N:%2i", compassOffset, gps_satellites);
      lcd.setCursor(0,1);
      lcd.printf("H:%3i C:%3i S%2.1f", int(ekfYawAngle), int(gps_course), ekfSpeed);
    }
    else if (mode == NORMAL_OPERATION_MODE){
      lcd.setCursor(0,0);
      lcd.printf("%i Manual Spd:%3.1f", mode, goalSpeed);
      lcd.setCursor(0,1);
      if (NORMAL_OPERATION_MODE_started) lcd.printf("H:%3i G:%3i S%2.1f", int(ekfYawAngle), int(goalAngle), ekfSpeed);
      else lcd.print("Butn+Up to Start");   
    }
    else if (mode == TURN_90_MODE){
      lcd.setCursor(0,0);
      lcd.printf("%i Turn90 Spd:%3.1f", mode, goalSpeed);
      lcd.setCursor(0,1);
      if (TURN_90_MODE_started) lcd.printf("H:%3i G:%3i S%2.1f", int(ekfYawAngle), int(goalAngle), ekfSpeed);
      else lcd.print("Butn+Up to Start");
    }
  }
}
/*End Display Modes*************************************************************************/
/***************************************************************************************/
