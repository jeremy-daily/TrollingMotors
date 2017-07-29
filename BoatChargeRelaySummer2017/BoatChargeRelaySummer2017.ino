/*
 * Added an optoislator to see the tachometer signals.
 */

const int greenLEDpin = 9;
const int redLEDpin = 3;
const int relayPin = 6;
const int alternatorPin = A0;
const int batteryPin = A2;
const int tachInputPin = 2;

uint32_t current_tach_count;
uint32_t previous_tach_count;

uint32_t display_millis;
uint32_t previous_millis;

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
boolean greenLEDstate = true;
boolean relayState = false;

void countTach(){
  previous_tach_count = 0;
  current_tach_count++;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(greenLEDpin,OUTPUT);
  pinMode(redLEDpin,OUTPUT);
  pinMode(relayPin,OUTPUT);
  pinMode(alternatorPin,INPUT);
  pinMode(batteryPin,INPUT);
  pinMode(tachInputPin,INPUT_PULLUP);
  
  digitalWrite(greenLEDpin,redLEDstate);
  digitalWrite(redLEDpin,greenLEDstate);
  digitalWrite(relayPin,relayState);

  Serial.begin(9600);
  delay(1000);
  attachInterrupt(0,countTach,FALLING);
}



void loop() {
  // put your main code here, to run repeatedly:
  
  alternatorReading = analogRead(alternatorPin);
  batteryReading = analogRead(batteryPin);

  alternatorMillivolts = map(alternatorReading,0,161,0,12870);
  batteryMillivolts = map(batteryReading,0,158,0,12660);

  if (millis() - display_millis > 250){
    display_millis = millis();
    Serial.print("Alt Reading: ");
    Serial.print(alternatorReading);
    Serial.print(", Battery Reading: ");
    Serial.print(batteryReading);
  
    Serial.print(", Alt milliVolts: ");
    Serial.print(alternatorMillivolts);
    Serial.print(", Battery milliVolts: ");
    Serial.print(batteryMillivolts);

    Serial.print(", Tach Count: ");
    Serial.println(current_tach_count);
  }
  
  if (millis() - previous_millis > 5000){
    previous_millis = millis();
    if (previous_tach_count == current_tach_count) current_tach_count = 0;
    
    if (current_tach_count > 200) 
    {
      relayState=true;
      greenLEDstate = true;
      redLEDstate = false;
      current_tach_count = 0;
      shutdownCount = millis();
    }
    else if ( alternatorMillivolts > 12700)
    {
      redLEDstate = true;
      relayState=true;
      current_tach_count = 0;
    }
    else
    {
      previous_tach_count = current_tach_count;
      greenLEDstate = false;
      relayState=false;
      redLEDstate = false;
    }
  }

  digitalWrite(relayPin,relayState);
  digitalWrite(greenLEDpin,greenLEDstate);
  digitalWrite(redLEDpin,redLEDstate);
}
