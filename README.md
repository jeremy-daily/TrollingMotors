# TrollingMotors
A project that runs 2 trolling motors on a boat with multiple microcontrollers on a CAN bus.

## Operation
Pressing the green button lets you change mode with an up and down of the joystick.

The red button stops the motor and sends the system into mode 0, which is off.

### Mode 0: OFF

### Mode 1: Normal 

### Mode 2: Figure 8

### Mode 3: Full

### Mode 4: Calibrate Compass



## Servos
Ues the following Arduino pins on the Teensy 3.2

D23 is the left servo.

D16 is the right motor.


## CAN Message
This section describes the interpretation and value for each of the CAN messages on the trolling motor system. All nodes are running at 500k CAN with 11-bit IDs.
### Control Message
* ID: 0x700
* Length: 1
* Meaning
  * Bit 0: upButtonState
  * Bit 1: downButtonState
  * Bit 2: leftButtonState
  * Bit 3: rightButtonState
  * Bit 4: pushButtonState
  * Bit 5: doubleClick
  * Bit 6: redButtonState
  * Bit 7: greenButtonState
  
This is the message sent by the Joystick or other controller every 25 ms. 

### Mode Message
* ID: 0x201
* Length: 2
* Meaning
 * Byte 0: number of modes of operation
 * Byte 1: current mode number

This messages is sent by the main controller as an informational message.
  
### Display Message 1
  ID: 0x211
  Length: 8
  All bytes are ascii characters to be displayed on the upper left character slots of a 16X2 character display.

### Display Message 2
  ID: 0x212
  Length: 8
  All bytes are ascii characters to be displayed on the upper right character slots of a 16X2 character display.

### Display Message 3
  ID: 0x221
  Length: 8
  All bytes are ascii characters to be displayed on the lower left character slots of a 16X2 character display.

### Display Message 4
  ID: 0x222
  Length: 8
  All bytes are ascii characters to be displayed on the lower right character slots of a 16X2 character display.

