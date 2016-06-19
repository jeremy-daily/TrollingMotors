# TrollingMotors
A project that runs 2 trolling motors on a boat with multiple microcontrollers on a CAN bus.

##CAN Message
This section describes the interpretationa adn value for each of the CAN messages on the trolling motor system. All nodes are running at 500k CAN with 11-bit IDs.
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
  All bytes are ascii characters to be displayed on the upper left scharacter slots of a 16X2 character display.
