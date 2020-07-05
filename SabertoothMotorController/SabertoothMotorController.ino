#include <FlexCAN.h>

#define MOTOR_STATUS_ID       0x020
#define MOTOR_COMMAND_ID      0x010

#define STANDBY_PIN  7

bool led_state;
elapsedMillis canRXtimer;
elapsedMillis canTXtimer;
uint8_t control_byte_1;
uint8_t control_byte_2;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(38400);
  Can0.begin(250000);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(STANDBY_PIN,OUTPUT);
  digitalWrite(STANDBY_PIN,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  CAN_message_t msg;
  if (Can0.read(msg)){
    if (msg.id == MOTOR_COMMAND_ID){
      canRXtimer = 0;
      led_state = !led_state;
      digitalWrite(LED_BUILTIN,led_state);
      control_byte_1 = 64  + int8_t(msg.buf[0]/2);
      control_byte_2 = 192 + int8_t(msg.buf[1]/2);
      Serial1.write(control_byte_1);
      Serial1.write(control_byte_2);
      Serial.printf("Motor Control: %3d %3d\n",control_byte_1, control_byte_2); 
  
    }
  }

  if (canRXtimer >= 1000) {
    control_byte_1 = 0;
    control_byte_2 = 0;
  }
  
  
  if (canTXtimer >= 100){
    canTXtimer = 0;
    msg.id = MOTOR_STATUS_ID;
    msg.buf[0] = control_byte_1;
    msg.buf[1] = control_byte_2;
    msg.len = 2;
    //Can0.write(msg);
    //Serial.printf("Motor Control: %3d %3d\n",control_byte_1, control_byte_2); 
  }
}
