#include <FlexCAN.h>
#include <kinetis_flexcan.h>

CAN_message_t  rxmsg;

CAN_filter_t allPassFilter;

boolean driveOn = false;
int led = 13;

elapsedMillis cantimeout;

void setup() {
  
  //start can network
 
  //start led and drive output
  pinMode(led, OUTPUT);
  pinMode(2, OUTPUT);
  //wait for serial to open
  digitalWrite(led,HIGH);
  digitalWrite(2,HIGH);
  delay(1000); 
  Serial.println("Starting Dodge Ram Charger Relay toggle using CAN message ID 0x312");
  Can0.begin(500000);

  //leave the first 4 mailboxes to use the default filter. Just change the higher ones
  allPassFilter.id=0;
  allPassFilter.ext=1;
  allPassFilter.rtr=0;
  for (uint8_t filterNum = 8; filterNum < 16;filterNum++){
    Can0.setFilter(allPassFilter,filterNum); 
    
  }
  
}
void loop() {
  //check if the engine is present and on
  if(Can0.available()){
    Can0.read(rxmsg);
    //Serial.printf("%03X\n",rxmsg.id);
    if(rxmsg.id == 0x312){
      if (rxmsg.buf[4] >= 0x0C)  cantimeout = 0;
      Serial.print("RPM: ");
      Serial.println(rxmsg.buf[4],HEX); 
    }
  }
  //check if engine has not been present for a time
  if( cantimeout >= 5000) driveOn = false;
  else driveOn = true;

  digitalWrite(led,driveOn);
  digitalWrite(2,driveOn);
}
