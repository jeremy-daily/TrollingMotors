
const int greenLEDpin = 2;
const int redLEDpin = 3;
const int relayPin = 6;
const int alternatorPin = A0;
const int batteryPin = A2;

int alternatorReading = 0;
int batteryReading = 0;

const int thresholdMillivolts = 11900;

int alternatorMillivolts = 0;
int batteryMillivolts = 0;

unsigned long int switchTime =0;
unsigned long int switchOffTime =0;
unsigned long int shutdownCount = 0;

const int switchDelay = 20000;
const int switchOffDelay = 5000;

boolean redLEDstate = true;
boolean greenLEDstate = false;
boolean relayState = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(greenLEDpin,OUTPUT);
  pinMode(redLEDpin,OUTPUT);
  pinMode(relayPin,OUTPUT);
  pinMode(alternatorPin,INPUT);
  pinMode(batteryPin,INPUT);
  
  digitalWrite(greenLEDpin,redLEDstate);
  digitalWrite(redLEDpin,greenLEDstate);
  digitalWrite(relayPin,relayState);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  alternatorReading = analogRead(alternatorPin);
  batteryReading = analogRead(batteryPin);

  alternatorMillivolts = map(alternatorReading,0,176,0,12700);
  batteryMillivolts = map(batteryReading,0,158,0,12660);
  
  Serial.print("Alt Reading: ");
  Serial.print(alternatorReading);
  Serial.print(", Battery Reading: ");
  Serial.print(batteryReading);

  Serial.print(", Alt milliVolts: ");
  Serial.print(alternatorMillivolts);
  Serial.print(", Battery milliVolts: ");
  Serial.println(batteryMillivolts);
  
  if (alternatorMillivolts >= thresholdMillivolts ){
    shutdownCount = millis();
    if ( (millis()-switchOffTime) > switchOffDelay){
      redLEDstate= false;
      greenLEDstate = true;
      relayState = true;
      switchTime = millis();
      switchOffTime = millis();
    }
    
  }
  if ( alternatorMillivolts<thresholdMillivolts) {
      
      if ( (millis()-switchTime) > switchDelay){
        redLEDstate= true;
        greenLEDstate = true;
        relayState = false;
        switchTime = millis();
        switchOffTime = millis();
      }
    }
  
  if (batteryMillivolts<11000){
     redLEDstate= true;
     greenLEDstate = false;
  }
  
  if (batteryMillivolts<9500){
     redLEDstate= false;
     greenLEDstate = false;
     relayState = false;
  }
  
  if (millis() - shutdownCount > 12000000){
     redLEDstate= false;
     greenLEDstate = false;
     relayState = false;
  }
  
  digitalWrite(relayPin,relayState);
  digitalWrite(greenLEDpin,greenLEDstate);
  digitalWrite(redLEDpin,redLEDstate);
}
