#include <Canbus.h>
#include <Wire.h>

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

//Sensing Pins
const int vSensePin  = 20;    // AD6 Voltage sense connected to voltage divider of 330k and 100k
// Save pins 18 and 19 for i2c (AD4 and AD5)
const int starVsensePin   = 15;    // AD1 right hand motor (port) side L IS sense
const int portVsensePin   = 14;    // AD0 right hand motor (port) side R IS sense

//Output Pins
//Port (left) has the gray Anderson connector
//Starboard (right) has the red Anderson connector
const int starServoPin = 8 ;
const int portServoPin = 9;


//Initialize state variables
int portMotor       = 0;   
int starMotor       = 0;

int minDuration   = 850; //microseconds for pulse width
int maxDuration    =2222; //microseconds for pulse width

int portZeroDuration  = 1818; //duration for the position for no motor movement
int starZeroDuration  = 1818; 

int portDuration      = portZeroDuration;
int starDuration      = starZeroDuration;

//Initialize Timing varialbles
unsigned long previousMillis250 = 0;
unsigned long previousMillis200 = 0;
unsigned long previousMillis20 = 0;

void setup()
{ 
  // initialize the digital pins.
  pinMode(vSensePin,   INPUT); 
  pinMode(portVsensePin, INPUT);
  pinMode(starVsensePin, INPUT);
  pinMode(portServoPin, OUTPUT);
  pinMode(starServoPin, OUTPUT);
  
  pulseServo(portServoPin,portZeroDuration);
  pulseServo(starServoPin,starZeroDuration);
  
  Wire.begin();
  Serial.begin(115200);

  //Initialize MCP2515 CAN controller at the specified speed
  if(Canbus.init(CANSPEED_500)) 
    Serial.println("CAN Init ok");
  else 
    Serial.println("Can't init CAN");
 
  
}
 
 
void loop()
{
  
  
  
  tCAN message;
   
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis20 > 20) { //Perform this every 20 milliseconds (50 Hz)
    previousMillis20 = currentMillis;   
    pulseServo(portServoPin,portDuration);
    pulseServo(starServoPin,starDuration);
        
      
  }
  if(currentMillis - previousMillis200 > 200) { //Perform this every 100 milliseconds
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
      //Serial.print(int (headingValue / 10));     // The whole number part of the heading
      //Serial.print(".");
      //Serial.print(int (headingValue % 10));     // The fractional part of the heading
      //Serial.println(" degrees");
    
    int vSenseReading       = analogRead(vSensePin);   
    int portVsenseReading   = analogRead(portVsensePin);   
    int starVsenseReading   = analogRead(starVsensePin);   
    
    message.id = 0x43D; //Made up Broadcast message
    message.header.length = 8;
    message.data[0] = highByte(vSenseReading);
    message.data[1] = lowByte(vSenseReading); 
    message.data[2] = highByte(portVsenseReading);
    message.data[3] = lowByte(portVsenseReading); 
    message.data[4] = highByte(starVsenseReading);
    message.data[5] = lowByte(starVsenseReading); 
    message.data[6] = headingData[0];
    message.data[7] = headingData[1]; 
                   
    mcp2515_send_message(&message);
  
    
    //Serial.print("Voltage Sense: ");
    //Serial.println(vSenseReading);
  }
  
  if (mcp2515_check_message){
     if(mcp2515_get_message(&message)){
        if (message.id==0x31A){ //Message from Joystick
           //Output data to Serial Console
           previousMillis250 = currentMillis;
           
           Serial.write("ID: ");
           Serial.print(message.id,HEX);
           Serial.print("  Data: ");
           for(int i = 0; i < 2; i++){
             Serial.print(message.data[i],HEX);
             Serial.print(" ");
           }
           Serial.println();
           
           //Parse the message
           
           byte starMotorByte   = message.data[0]; // a value from 0 to 255
           byte portMotorByte   = message.data[1];;
                      
           
           int portMotor = portMotorByte - 100;
           int starMotor = starMotorByte - 100;
           
           portDuration = map(portMotor,0,100,portZeroDuration,minDuration);
           starDuration = map(starMotor,0,100,starZeroDuration,minDuration);
           portDuration = constrain(portDuration, minDuration,maxDuration);
           starDuration = constrain(starDuration, minDuration,maxDuration);
           
           //Serial.print("Duration: ");
           //Serial.println(starDuration);
          }
       }
  }
  if (currentMillis - previousMillis250 > 250){
      portDuration = portZeroDuration;
      starDuration = starZeroDuration;
      //Serial.println("No CAN Message detected.");
  }
} //end loop()

void pulseServo(int PIN, int duration){
  // Turn off interupts and send out a precise pulse to reduce jitter.
  duration = constrain(duration, minDuration,maxDuration);
  //Serial.print("Duration: ");
  //Serial.println(starDuration);
  
  long start = micros();
  digitalWrite(PIN, HIGH);
  for (int i = 0; i<maxDuration; i++){
    if (micros() - start > duration){
      break;
    }
   }
   digitalWrite(PIN, LOW);
  
}
