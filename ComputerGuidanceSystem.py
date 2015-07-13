import struct
import sys
import os
import msvcrt
from   ctypes import *
from   ctypes.wintypes import HWND
from   array import *
from   time import *

#-----------------------------------------------------------------------------------------------------
# RP1210B   RP1210_SendCommand Defines (From RP1210B Document)
#-----------------------------------------------------------------------------------------------------

RP1210_Reset_Device                               =                   0
RP1210_Set_All_Filters_States_to_Pass             =                   3
RP1210_Set_Message_Filtering_For_J1939            =                   4
RP1210_Set_Message_Filtering_For_CAN              =                   5
RP1210_Set_Message_Filtering_For_J1708            =                   7
RP1210_Set_Message_Filtering_For_J1850            =                   8
RP1210_Set_Message_Filtering_For_ISO15765         =                   9
RP1210_Generic_Driver_Command                     =                  14
RP1210_Set_J1708_Mode                             =                  15
RP1210_Echo_Transmitted_Messages                  =                  16
RP1210_Set_All_Filters_States_to_Discard          =                  17
RP1210_Set_Message_Receive                        =                  18
RP1210_Protect_J1939_Address                      =                  19
RP1210_Set_Broadcast_For_J1708                    =                  20
RP1210_Set_Broadcast_For_CAN                      =                  21
RP1210_Set_Broadcast_For_J1939                    =                  22
RP1210_Set_Broadcast_For_J1850                    =                  23
RP1210_Set_J1708_Filter_Type                      =                  24
RP1210_Set_J1939_Filter_Type                      =                  25
RP1210_Set_CAN_Filter_Type                        =                  26
RP1210_Set_J1939_Interpacket_Time                 =                  27
RP1210_SetMaxErrorMsgSize                         =                  28
RP1210_Disallow_Further_Connections               =                  29
RP1210_Set_J1850_Filter_Type                      =                  30
RP1210_Release_J1939_Address                      =                  31
RP1210_Set_ISO15765_Filter_Type                   =                  32
RP1210_Set_Broadcast_For_ISO15765                 =                  33
RP1210_Set_ISO15765_Flow_Control                  =                  34
RP1210_Clear_ISO15765_Flow_Control                =                  35
RP1210_Set_ISO15765_Link_Type                     =                  36
RP1210_Set_J1939_Baud                             =                  37
RP1210_Set_ISO15765_Baud                          =                  38
RP1210_Set_BlockTimeout                           =                 215
RP1210_Set_J1708_Baud                             =                 305

#-----------------------------------------------------------------------------------------------------
# RP1210B Constants - Check RP1210B document for any updates.
#-----------------------------------------------------------------------------------------------------

CONNECTED                                         =             1  # Connection state = Connected 
NOT_CONNECTED                                     =            -1  # Connection state = Disconnected

FILTER_PASS_NONE                                  =             0  # Filter state = DISCARD ALL MESSAGES
FILTER_PASS_SOME                                  =             1  # Filter state = PASS SOME (some filters)
FILTER_PASS_ALL                                   =             2  # Filter state = PASS ALL

NULL_WINDOW                                       =             0  # Windows 3.1 is no longer supported.

BLOCKING_IO                                       =             1  # For Blocking calls to send/read.
NON_BLOCKING_IO                                   =             0  # For Non-Blocking calls to send/read.
BLOCK_INFINITE                                    =             0  # For Non-Blocking calls to send/read.

BLOCK_UNTIL_DONE                                  =             0  # J1939 Address claim, wait until done
RETURN_BEFORE_COMPLETION                          =             2  # J1939 Address claim, don't wait

CONVERTED_MODE                                    =             1  # J1708 RP1210Mode="Converted"
RAW_MODE                                          =             0  # J1708 RP1210Mode="Raw"

MAX_J1708_MESSAGE_LENGTH                          =           508  # Maximum size of J1708 message (+1)
MAX_J1939_MESSAGE_LENGTH                          =          1796  # Maximum size of J1939 message (+1)
MAX_ISO15765_MESSAGE_LENGTH                       =          4108  # Maximum size of ISO15765 message (+1)

ECHO_OFF                                          =          0x00  # EchoMode
ECHO_ON                                           =          0x01  # EchoMode

RECEIVE_ON                                        =          0x01  # Set Message Receive
RECEIVE_OFF                                       =          0x00  # Set Message Receive

ADD_LIST                                          =          0x01  # Add a message to the list.
VIEW_B_LIST                                       =          0x02  # View an entry in the list.
DESTROY_LIST                                      =          0x03  # Remove all entries in the list.
REMOVE_ENTRY                                      =          0x04  # Remove a specific entry from the list.
LIST_LENGTH                                       =          0x05  # Returns number of items in list.

FILTER_PGN                                        =          0x01  # Setting of J1939 filters
FILTER_PRIORITY                                   =          0x02  # Setting of J1939 filters
FILTER_SOURCE                                     =          0x04  # Setting of J1939 filters
FILTER_DESTINATION                                =          0x08  # Setting of J1939 filters
FILTER_INCLUSIVE                                  =          0x00  # FilterMode
FILTER_EXCLUSIVE                                  =          0x01  # FilterMode

SILENT_J1939_CLAIM                                =          0x00  # Claim J1939 Address
PASS_J1939_CLAIM_MESSAGES                         =          0x01  # Claim J1939 Address

CHANGE_BAUD_NOW                                   =          0x00  # Change Baud
MSG_FIRST_CHANGE_BAUD                             =          0x01  # Change Baud
RP1210_BAUD_9600                                  =          0x00  # Change Baud
RP1210_BAUD_19200                                 =          0x01  # Change Baud
RP1210_BAUD_38400                                 =          0x02  # Change Baud
RP1210_BAUD_57600                                 =          0x03  # Change Baud
RP1210_BAUD_125k                                  =          0x04  # Change Baud
RP1210_BAUD_250k                                  =          0x05  # Change Baud
RP1210_BAUD_500k                                  =          0x06  # Change Baud
RP1210_BAUD_1000k                                 =          0x07  # Change Baud

STANDARD_CAN                                      =          0x00  # Filters
EXTENDED_CAN                                      =          0x01  # Filters

STANDARD_CAN_ISO15765_EXTENDED                    =          0x02  # 11-bit with ISO15765 extended address
EXTENDED_CAN_ISO15765_EXTENDED                    =          0x03  # 29-bit with ISO15765 extended address
STANDARD_MIXED_CAN_ISO15765                       =          0x04  # 11-bit identifier with mixed addressing
ISO15765_ACTUAL_MESSAGE                           =          0x00  # ISO15765 ReadMessage - type of data
ISO15765_CONFIRM                                  =          0x01  # ISO15765 ReadMessage - type of data
ISO15765_FF_INDICATION                            =          0x02  # ISO15765 ReadMessage - type of data

LINKTYPE_GENERIC_CAN                              =          0x00  # Set_ISO15765_Link_Type argument
LINKTYPE_J1939_ISO15765_2_ANNEX_A                 =          0x01  # Set_ISO15765_Link_Type argument
LINKTYPE_J1939_ISO15765_3                         =          0x02  # Set_ISO15765_Link_Type argument

#-----------------------------------------------------------------------------------------------------
# Local #define Definitions
#-----------------------------------------------------------------------------------------------------

J1939_GLOBAL_ADDRESS                              =           255
J1939_OFFBOARD_DIAGNOSTICS_TOOL_1                 =           249
J1587_OFFBOARD_DIAGNOSTICS_TOOL_1                 =           172

# ----------------------------------------------------------------------------------------------------
# PrintRP1210Error Dictionary data Structure
# 
RP1210Errors = {
    0  : "NO_ERRORS"                                        ,
    128: "ERR_DLL_NOT_INITIALIZED"                          ,
    129: "ERR_INVALID_CLIENT_ID"                            ,
    130: "ERR_CLIENT_ALREADY_CONNECTED"                     ,
    131: "ERR_CLIENT_AREA_FULL"                             ,
    132: "ERR_FREE_MEMORY"                                  ,
    133: "ERR_NOT_ENOUGH_MEMORY"                            ,
    134: "ERR_INVALID_DEVICE"                               ,
    135: "ERR_DEVICE_IN_USE"                                ,
    136: "ERR_INVALID_PROTOCOL"                             ,
    137: "ERR_TX_QUEUE_FULL"                                ,
    138: "ERR_TX_QUEUE_CORRUPT"                             ,
    139: "ERR_RX_QUEUE_FULL"                                ,
    140: "ERR_RX_QUEUE_CORRUPT"                             ,
    141: "ERR_MESSAGE_TOO_LONG"                             ,
    142: "ERR_HARDWARE_NOT_RESPONDING"                      ,
    143: "ERR_COMMAND_NOT_SUPPORTED"                        ,
    144: "ERR_INVALID_COMMAND"                              ,
    145: "ERR_TXMESSAGE_STATUS"                             ,
    146: "ERR_ADDRESS_CLAIM_FAILED"                         ,
    147: "ERR_CANNOT_SET_PRIORITY"                          ,
    148: "ERR_CLIENT_DISCONNECTED"                          ,
    149: "ERR_CONNECT_NOT_ALLOWED"                          ,
    150: "ERR_CHANGE_MODE_FAILED"                           ,
    151: "ERR_BUS_OFF"                                      ,
    152: "ERR_COULD_NOT_TX_ADDRESS_CLAIMED"                 ,
    153: "ERR_ADDRESS_LOST"                                 ,
    154: "ERR_CODE_NOT_FOUND"                               ,
    155: "ERR_BLOCK_NOT_ALLOWED"                            ,
    156: "ERR_MULTIPLE_CLIENTS_CONNECTED"                   ,
    157: "ERR_ADDRESS_NEVER_CLAIMED"                        ,
    158: "ERR_WINDOW_HANDLE_REQUIRED"                       ,
    159: "ERR_MESSAGE_NOT_SENT"                             ,
    160: "ERR_MAX_NOTIFY_EXCEEDED"                          ,
    161: "ERR_MAX_FILTERS_EXCEEDED"                         ,
    162: "ERR_HARDWARE_STATUS_CHANGE"                       ,
    202: "ERR_INI_FILE_NOT_IN_WIN_DIR"                      ,
    204: "ERR_INI_SECTION_NOT_FOUND"                        ,
    205: "ERR_INI_KEY_NOT_FOUND"                            ,
    206: "ERR_INVALID_KEY_STRING"                           ,
    207: "ERR_DEVICE_NOT_SUPPORTED"                         ,
    208: "ERR_INVALID_PORT_PARAM"                           ,
    213: "ERR_COMMAND_TIMED_OUT"                            ,
    220: "ERR_OS_NOT_SUPPORTED"                             ,
    222: "ERR_COMMAND_QUEUE_IS_FULL"                        ,
    224: "ERR_CANNOT_SET_CAN_BAUDRATE"                      ,
    225: "ERR_CANNOT_CLAIM_BROADCAST_ADDRESS"               ,
    226: "ERR_OUT_OF_ADDRESS_RESOURCES"                     ,
    227: "ERR_ADDRESS_RELEASE_FAILED"                       ,
    230: "ERR_COMM_DEVICE_IN_USE"                           ,
    441: "ERR_DATA_LINK_CONFLICT"                           ,
    453: "ERR_ADAPTER_NOT_RESPONDING"                       ,
    454: "ERR_CAN_BAUD_SET_NONSTANDARD"                     ,
    455: "ERR_MULTIPLE_CONNECTIONS_NOT_ALLOWED_NOW"         ,
    456: "ERR_J1708_BAUD_SET_NONSTANDARD"                   ,
    457: "ERR_J1939_BAUD_SET_NONSTANDARD"                   }

#
#
#********************************************************************************
# Global Variables
#********************************************************************************
#
# RP1210_ReadDetailedVersion
#
chAPIVersionInfo    = (c_char*17)()
chDLLVersionInfo    = (c_char*17)()
chFWVersionInfo     = (c_char*17)()
szAPI               = (c_char)(100)
szDLL               = (c_char)(100)
szFW                = (c_char)(100)

#
# Connection Related Variables
#

iAdapterID          = c_short() 
szDLLName           = (c_char*100)()

iProtocolID         = c_short()
szProtocolName      = (c_char*100)()

iDeviceID           = c_short()

nClientID           = c_short()
nRetVal             = c_short()

ucTxRxBuffer        = (c_char*2000)()

#
#
#********************************************************************************
# Global Functions
#********************************************************************************
#
#

#-----------------------------------------------------------------------------------------------------
# Print a received CAN message.
#-----------------------------------------------------------------------------------------------------

def PrintRxCANMessage( nRetVal, ucTxRxBuffer ) :
    '''This function prints a received CAN message to the screen.'''

    iTS        = c_int  (0)
    ucCANType  = c_char (0)
    iCANType   = c_int  (0)

    iCANID     = c_int  (0)

    nDataBytes = c_short(0)

    iTS        = int(struct.unpack('>L',(ucTxRxBuffer[0]+ucTxRxBuffer[1]+ucTxRxBuffer[2]+ucTxRxBuffer[3]))[0])

    ucCANType  = c_char( struct.unpack( 'B', ucTxRxBuffer[4] )[0])
    iCANType   = bytes( ucCANType )[0]

    if iCANType == 0 :
        szCANType = "STD"
        iCANID    = int(struct.unpack('>H',(ucTxRxBuffer[5]+ucTxRxBuffer[6]))[0])
        iDataIdx  = 7
    else :
        szCANType = "EXT"
        iCANID    = int(struct.unpack('>L',(ucTxRxBuffer[5]+ucTxRxBuffer[6]+ucTxRxBuffer[7]+ucTxRxBuffer[8]))[0])
        iDataIdx  = 9
    #if

    nDataBytes = nRetVal - iDataIdx

    print( "Rx CAN TS=[%d]"   % iTS         , end=" " )
    print(      "TYPE=[%s]"   % szCANType   , end=" " )
    print(    "CANID=[%0X]"   % iCANID                )
    print( ""                               , end="\t")
    print( "DATA-HEX"                       , end=""  )

    for ucByte in ucTxRxBuffer[iDataIdx:nRetVal] :
         print("[%02X]" % ucByte, end="" )
    #for

    print("")

#def PrintCANMessage

#-----------------------------------------------------------------------------------------------------
# Process a received CAN message.
#-----------------------------------------------------------------------------------------------------

def ProcessRxCANMessage( nRetVal, ucTxRxBuffer ) :
    '''This function prints a received CAN message to the screen.'''

    iTS        = c_int  (0)
    ucCANType  = c_char (0)
    iCANType   = c_int  (0)

    iCANID     = c_int  (0)

    nDataBytes = c_short(0)

    iTS        = int(struct.unpack('>L',(ucTxRxBuffer[0]+ucTxRxBuffer[1]+ucTxRxBuffer[2]+ucTxRxBuffer[3]))[0])

    ucCANType  = c_char( struct.unpack( 'B', ucTxRxBuffer[4] )[0])
    iCANType   = bytes( ucCANType )[0]

    if iCANType == 0 :
        szCANType = "STD"
        iCANID    = int(struct.unpack('>H',(ucTxRxBuffer[5]+ucTxRxBuffer[6]))[0])
        iDataIdx  = 7
    else :
        szCANType = "EXT"
        iCANID    = int(struct.unpack('>L',(ucTxRxBuffer[5]+ucTxRxBuffer[6]+ucTxRxBuffer[7]+ucTxRxBuffer[8]))[0])
        iDataIdx  = 9
    #if
    return iCANID,ucTxRxBuffer[iDataIdx:nRetVal]
    
   


def PrintTxCANMessage( nRetVal, ucTxRxBuffer ) :
    '''This function prints a transmitted CAN message to the screen.'''

    ucCANType  = c_char (0)
    iCANType   = c_int  (0)

    iCANID     = c_int  (0)

    nDataBytes = c_short(0)

    ucCANType  = c_char( struct.unpack( 'B', ucTxRxBuffer[0] )[0])
    iCANType   = bytes( ucCANType )[0]

    if iCANType == 0 :
        szCANType = "STD"
        iCANID    = int(struct.unpack('>L',(b'\x00'+ucTxRxBuffer[1]+ucTxRxBuffer[2]+ucTxRxBuffer[3]))[0])
        iDataIdx  = 4
    else :
        szCANType = "EXT"
        iCANID    = int(struct.unpack('>L',(ucTxRxBuffer[1]+ucTxRxBuffer[2]+ucTxRxBuffer[3]+ucTxRxBuffer[4]))[0])
        iDataIdx  = 5
    #if

    nDataBytes = nRetVal - iDataIdx

    print( "Tx CAN TYPE=[%s]"   % szCANType   , end=" " )
    print(      "CANID=[%0X]"   % iCANID                )
    print( ""                                 , end="\t")
    print( "DATA-HEX"                         , end=""  )

    for ucByte in ucTxRxBuffer[iDataIdx:nRetVal] :
         print("[%02X]" % ucByte, end="" )
    #for

    print("")

#def PrintTxCANMessage

#-----------------------------------------------------------------------------------------------------
#  Send a CAN Message
#-----------------------------------------------------------------------------------------------------

def SendCANSpeedMessage( nClientID,rightMotor,leftMotor ) :
   
    nDataBytes = c_short(0)

    ucTxRxBuffer[0] = 0x00        # STD CAN
    ucTxRxBuffer[0] = 0x00        # CANID 0x18EAFFF9 - Pri ExtDP/DP
    ucTxRxBuffer[1] = 0x03        # PF=1
    ucTxRxBuffer[2] = 0x1A        # PS=Global Address
    ucTxRxBuffer[3] = struct.pack('B',rightMotor)       # LSB First of PGN65260 0x00FEEC
    ucTxRxBuffer[4] = struct.pack('B',leftMotor)        # Middle
    #ucTxRxBuffer[7] = 0x00        # High Byte
    
    nDataBytes = 5;
  
    nRetVal = RP1210_SendMessage( c_short( nClientID ), byref( ucTxRxBuffer ), c_short( nDataBytes ), c_short( 0 ), c_short( NON_BLOCKING_IO ) )
    return
    
    #if  nRetVal != 0 :
    #    print("RP1210_SendMessage( CAN ) returns %i" % nRetVal )
    #else :
    #    PrintTxCANMessage( nDataBytes, ucTxRxBuffer )
    #if

#def SendCANRequestMessage

#
#        
#-----------------------------------------------------------------------------------------------------
# main() Code Body
#-----------------------------------------------------------------------------------------------------
#

iAdapterID = 2

while iAdapterID > 6 or iAdapterID < 1 :
   # print( "                                                                         " )
   # print( "------------------------------------------------------------------------ " )
   # print( " Select Adapter To Use                                                   " )
   # print( "------------------------------------------------------------------------ " )
   # print( "                                                                         " )
   # print( " 1 = DPA 4 Plus Single-Application           ( DLL Name =  DPA4PSA.DLL ) " )
   # print( " 2 = DPA 4 Plus Multi-Application            ( DLL Name =  DPA4PMA.DLL ) " )
   # print( " 3 = DPA 5      Single-Application           ( DLL Name = DGDPA5SA.DLL ) " )
   # print( " 4 = DPA 5      Multi-Application            ( DLL Name = DGDPA5MA.DLL ) " )
   # print( " 5 = DPA 4 Plus And Prior DPAs               ( DLL Name = DG121032.DLL ) " )
   # print( " 6 = Allison DR Drivers ( DPA 4 Plus/DPA 5 ) ( DLL Name = DR121032.DLL ) " )
   # print( "                                                                         " )
   
   #iAdapterID = int( input( "-> " ) )

   if iAdapterID == 1:
      szDLLName = "DPA4PSA.DLL" 
      break
   #if
   if iAdapterID == 2: 
      szDLLName =   'DPA4PMA.DLL' 
      break
   #if
   if iAdapterID == 3: 
      szDLLName =  "DGDPA5SA.DLL" 
      break
   #if
   if iAdapterID == 4: 
      szDLLName =  "DGDPA5MA.DLL" 
      break
   #if
   if iAdapterID == 5: 
      szDLLName =  "DG121032.DLL" 
      break
   #if
   if iAdapterID == 6: 
      szDLLName =  "DR121032.DLL" 
      break
   #if

# while

#-----------------------------------------------------------------------------------------------------
# Get the Protocol name to use.
#-----------------------------------------------------------------------------------------------------

iProtocolID = 3

while iProtocolID < 1 or iProtocolID > 5 :
   print( "                                                                     " )
   print( "---------------------------------------------------------------------" )
   print( " Selection Protocol To Use                                           " )
   print( "---------------------------------------------------------------------" )
   print( "                                                                     " )
   print( " 1 = J1708                                                           " )
   print( " 2 = J1939                                                           " )
   print( " 3 = J1939:Baud=Auto  ( Automatic Baud Detect )                      " )
   print( " 4 = CAN                                                             " )
   print( " 5 = CAN:Baud=Auto    ( Automatic Baud Detect )                      " )
   print( "                                                                     " )
   print(  )
   
   #iProtocolID = int( input( "-> " ) )

   if iProtocolID == 1:
      szProtocolName = bytes( "J1708" , 'ascii' )
      break
   #if
   if iProtocolID == 2: 
      szProtocolName = bytes( "J1939" , 'ascii' )
      break
   #if
   if iProtocolID == 3: 
      szProtocolName = bytes( "J1939:Baud=Auto" , 'ascii' )
      break
   #if
   if iProtocolID == 4: 
      szProtocolName = bytes( 'CAN' , 'ascii' )
      break
   #if
   if iProtocolID == 5: 
      szProtocolName = bytes( "CAN:Baud=Auto" , 'ascii' )
      break
   #if

# while

#-----------------------------------------------------------------------------------------------------
# Get the DeviceID to use.
#-----------------------------------------------------------------------------------------------------

print( "                                                                      " )
print( "--------------------------------------------------------------------- " )
print( " Select Device To Use                                                 " )
print( "--------------------------------------------------------------------- " )
print( "                                                                      " )

if iAdapterID == 1 or iAdapterID == 2 :
   print( " Common DPA 4 Plus Single-Application/Multi-Application Devices   " )
   print( "     1   =  DPA 4 Plus - USB                                      " )
#if
if iAdapterID == 3 or iAdapterID == 4 :
   print( " Common DPA 5 Single-Application/Multi-Application Devices        " )
   print( "     1   =  DPA 5 - USB                                           " )
#if
if iAdapterID == 5 :
   print( " Common DG121032.ini Devices                                      " )
   print( "     150 =  DPA 4/4 Plus USB                                      " )
   print( "     101 =  DPA II/II+/III+ Serial Using COM1                     " )
   print( "     102 =  DPA II/II+/III+ Serial Using COM2                     " )
#if
if iAdapterID == 6 :
   print( " Common DR121032.ini Devices                                      " )
   print( "     150 =  DPA 4 USB                                             " )
   print( "     151 =  DPA 4 Plus USB and DPA 5 USB                          " )
#if

#print( "                                                                      " )
#print( " These are the most common DG Technologies devices based on the DLL   " )
#print( " that was selected.  Other device numbers may exist in that INI file  " )
#print( " such as Bluetooth entries for the DPA 5 ( beginning at 160 ).        " )
#print( " Please enter the device number you would like to use.                " )
#print( "                                                                      " )

#iDeviceID = int( input( "-> " ) )
 
#-----------------------------------------------------------------------------------------------------
#  Load the RP1210 Library, assign function pointers.
#-----------------------------------------------------------------------------------------------------

hRP1210DLL = windll.LoadLibrary( 'DPA4PMA.DLL'  )
print('The result from loading the RP1210 driver library is:')
print(hRP1210DLL)
#try
        
#-----------------------------------------------------------------------------------------------------
# RP1210_ClientConnect
#-----------------------------------------------------------------------------------------------------
# typedef short (WINAPI *fxRP1210_ClientConnect)       ( HWND, short, char *, long, long, short );
    
prototype                   = WINFUNCTYPE( c_short, HWND, c_short, c_char_p, c_long, c_long, c_short)
RP1210_ClientConnect        = prototype( ( "RP1210_ClientConnect", hRP1210DLL ) )
    
#-----------------------------------------------------------------------------------------------------
# RP1210_ClientDisconnect
#-----------------------------------------------------------------------------------------------------
# typedef short (WINAPI *fxRP1210_ClientDisconnect)    ( short                                  );

prototype                   = WINFUNCTYPE( c_short, c_short )
RP1210_ClientDisconnect     = prototype( ( "RP1210_ClientDisconnect", hRP1210DLL ) )

#-----------------------------------------------------------------------------------------------------
# RP1210_SendMessage
#-----------------------------------------------------------------------------------------------------
# typedef short (WINAPI *fxRP1210_SendMessage)         ( short, char*, short, short, short      );

prototype                   = WINFUNCTYPE( c_short, c_short,  POINTER( c_char*2000 ), c_short, c_short, c_short      )
RP1210_SendMessage          = prototype( ("RP1210_SendMessage", hRP1210DLL ) )

#-----------------------------------------------------------------------------------------------------
# RP1210_ReadMessage
#-----------------------------------------------------------------------------------------------------
# typedef short (WINAPI *fxRP1210_ReadMessage)         ( short, char*, short, short             );

prototype                   = WINFUNCTYPE( c_short, c_short, POINTER( c_char*2000 ), c_short, c_short             )
RP1210_ReadMessage          = prototype( ("RP1210_ReadMessage", hRP1210DLL ) )

#-----------------------------------------------------------------------------------------------------
# RP1210_SendCommand
#-----------------------------------------------------------------------------------------------------
# typedef short (WINAPI *fxRP1210_SendCommand)         ( short, short, char*, short             );

prototype                   = WINFUNCTYPE( c_short, c_short, c_short, POINTER( c_char*2000 ), c_short             )
RP1210_SendCommand          = prototype( ("RP1210_SendCommand", hRP1210DLL ) )

#-----------------------------------------------------------------------------------------------------
# RP1210_ReadVersion
#-----------------------------------------------------------------------------------------------------
# typedef short (WINAPI *fxRP1210_ReadVersion)         ( char*, char*, char*, char*             );

prototype                   = WINFUNCTYPE( c_short, c_char_p, c_char_p, c_char_p, c_char_p             )
RP1210_ReadVersion          = prototype( ("RP1210_ReadVersion", hRP1210DLL ) )

#-----------------------------------------------------------------------------------------------------
# RP1210_ReadDetailedVersion
#-----------------------------------------------------------------------------------------------------
# typedef short (WINAPI *fxRP1210_ReadDetailedVersion) ( short, char*, char*, char*             );

prototype                   = WINFUNCTYPE( c_short, c_short, POINTER(c_char*17), POINTER(c_char*17), POINTER(c_char*17) )
RP1210_ReadDetailedVersion  = prototype( ("RP1210_ReadDetailedVersion", hRP1210DLL ) )

#-----------------------------------------------------------------------------------------------------
# RP1210_GetHardwareStatus
#-----------------------------------------------------------------------------------------------------
# typedef short (WINAPI *fxRP1210_GetHardwareStatus)   ( short, char*, short, short             );

prototype                   = WINFUNCTYPE( c_short, c_short, c_char_p, c_short, c_short             )
RP1210_GetHardwareStatus    = prototype( ("RP1210_GetHardwareStatus", hRP1210DLL ) )

#-----------------------------------------------------------------------------------------------------
# RP1210_GetErrorMsg
#-----------------------------------------------------------------------------------------------------
# typedef short (WINAPI *fxRP1210_GetErrorMsg)         ( short, char*                           );

prototype                   = WINFUNCTYPE( c_short, c_short, c_char_p                           )
RP1210_GetErrorMsg          = prototype( ("RP1210_GetErrorMsg", hRP1210DLL ) )

#-----------------------------------------------------------------------------------------------------
# RP1210_GetLastErrorMsg
#-----------------------------------------------------------------------------------------------------
# typedef short (WINAPI *fxRP1210_GetLastErrorMsg)     ( short, int *, char*, short             );

prototype                   = WINFUNCTYPE( c_short, c_void_p, c_char_p, c_short             )
RP1210_GetLastErrorMsg      = prototype( ("RP1210_GetLastErrorMsg", hRP1210DLL ) )

#-----------------------------------------------------------------------------------------------------
# Connect to Device
#-----------------------------------------------------------------------------------------------------

#print( "Attempting connect to DLL [%s], DeviceID [%d], Protocol [%s]" %( szDLLName, int(iDeviceID), str( szProtocolName, 'ascii' ) ) )

nClientID = RP1210_ClientConnect( HWND(None),  1 , bytes('CAN:Baud=Auto','ascii'), 0, 0, 0  )

print('The Client ID is: %i' %nClientID )

if nClientID > 127:
   print('There was an error calling RP1210_ClientConnect(): %s' % RP1210Errors[nClientID] )
   sys.exit( int( nClientID ) )
#if

#-----------------------------------------------------------------------------------------------------
# Call RP1210_ReadDetailedVersion to get DLL, API, FW versions.
#-----------------------------------------------------------------------------------------------------

nRetVal = RP1210_ReadDetailedVersion( c_short( nClientID ), byref( chAPIVersionInfo ), byref( chDLLVersionInfo ), byref( chFWVersionInfo ) )

if nRetVal == 0 :
   szAPI = str( chAPIVersionInfo, 'ascii' )
   szDLL = str( chDLLVersionInfo, 'ascii' )
   szFW  = str( chFWVersionInfo , 'ascii' )

   print('DLL = %s' % szDLL  )
   print('API = %s' % szAPI  )
   print('FW  = %s' % szFW   )
else :   
   print("ReadDetailedVersion fails with a return value of  %i" %nRetVal )
#if

#-----------------------------------------------------------------------------------------------------
# Set all filters to pass.  This allows messages to be read.
#-----------------------------------------------------------------------------------------------------

nRetVal = RP1210_SendCommand( c_short( RP1210_Set_All_Filters_States_to_Pass ), c_short( nClientID ), None, 0 )

if nRetVal == 0 :
   print("RP1210_Set_All_Filters_States_to_Pass - SUCCESS" )
else :
   print('RP1210_Set_All_Filters_States_to_Pass returns %i' %nRetVal )
#if

#-----------------------------------------------------------------------------------------------------
# If we are on J1939, claim address F9/249.
#-----------------------------------------------------------------------------------------------------

ProtocolName = str( szProtocolName, 'ascii' )


#-----------------------------------------------------------------------------------------------------
# Set a main read/process loop.  Send message every .2 seconds.
#-----------------------------------------------------------------------------------------------------

LastTimeRequestsSent = time()
LastTimeDisplay = time() 
networkData={}
networkData['Heading']=0
goal=0
speed=0
heading=0
rightMotor=speed+100
leftMotor=speed+100
adjust=0
heading1=0
gpsSpeed=0
gpsAngle=0
gpsSats=0
gpsFix=False
rightTurn=False
leftTurn=False
infinityTurn=False
data1=b'000000'
memorySize=2000
differenceList=[]
for c in range(memorySize):
    differenceList.append(0)
firstTime=True 
degreeCount=0  
infinityCount=0
infinityTurn=False
lastTurnTime=time()
turnAroundCount=0
leftTurnAround=False
rightTurnAround=False
i=0
 
while True:

    if msvcrt.kbhit(): #and msvcrt.getch().decode() == chr(27): #press Escape to exit

        #print('You pressed a key:')
        #keyPressed = msvcrt.getch().decode() 
        keyPressed = msvcrt.getch()
        
        try:
            print(keyPressed.decode())
            if keyPressed==b'\x1b': #Escape
                print('Bye Bye')
                aborted = True
                break
            elif keyPressed.decode()==b'q': #quit
                print('Bye Bye')
                aborted = True
                break
            elif keyPressed==b'\x20': #Space
                #reset Direction and speed
                speed=0
                adjust=0
                goal = heading
            elif keyPressed==b'\x30': #zero
                speed=100
            elif keyPressed==b'\x31': 
                speed=10
            elif keyPressed==b'\x32': 
                speed=20
            elif keyPressed==b'\x33': 
                speed=30
            elif keyPressed==b'\x34': 
                speed=40
            elif keyPressed==b'\x35': 
                speed=50
            elif keyPressed==b'\x36': 
                speed=60
            elif keyPressed==b'\x37': 
                speed=70
            elif keyPressed==b'\x38': 
                speed=80
            elif keyPressed==b'\x39': 
                speed=90
            elif keyPressed.decode()=='f': #faster
                speed+=1
            elif keyPressed.decode()=='u': #speed up
                speed+=1
            elif keyPressed.decode()=='d': #slow down
                speed-=1
            elif keyPressed.decode()=='n': #north
                goal=345
            elif keyPressed.decode()=='s': #south
                goal=165
            elif keyPressed.decode()=='e': #east
                goal=75
            elif keyPressed.decode()=='w': #west
                goal=255
            elif keyPressed.decode()=='l': #left
                leftTurn=True
            elif keyPressed.decode()=='.': #right by 1 degree
                goal+=1
            elif keyPressed.decode()=='r': #right
                rightTurn=True
            elif keyPressed.decode()=='i': #Figure 8 (infinity)
                infinityTurn=True
            elif keyPressed.decode()==',': #left
                goal-=1
            elif keyPressed.decode()=='h': #heading
                goal=heading
            elif keyPressed.decode()=='t': #turn around left
                leftTurnAround=True
            elif keyPressed.decode()=='y': #turn around right
                rightTurnAround=True
          
          
            if keyPressed.decode() not in ['u','d','0','1','2','3','4','5','6','7','8','9']:
                if keyPressed.decode() != 'r':
                    rightTurn=False
                if keyPressed.decode() != 'l':
                    leftTurn=False
                if keyPressed.decode() != 'i':
                    infinityTurn = False
                if keyPressed.decode() != 't':
                    leftTurnAround = False
                if keyPressed.decode() != 'y':
                    rightTurnAround = False
            

            
        except UnicodeDecodeError:
            pass
            
           
    nRetVal = RP1210_ReadMessage( c_short( nClientID ), byref( ucTxRxBuffer ), c_short( 2000 ), c_short( NON_BLOCKING_IO ) )

    if nRetVal > 0 :
        id,data=ProcessRxCANMessage( nRetVal, ucTxRxBuffer )
        #print('ID: %X' %id, end='\t')
        #print('Data:', end='\t')
        #print(data)
        
        if id==int('43c',16):
            heading1=(data[0]*256+data[1])/10.
        elif id==int('43e',16):
            data1=data
            heading=(data[0]*256+data[1])/10.
            gpsSpeed=(data[2]*256 + data[3])*1.15078
            gpsAngle=(data[4]*256 + data[5])
            gpsSats=int(data[6])
            gpsFix=data[7]
            if firstTime:
                goal=heading
                firstTime=False
      #if

    elif nRetVal < 0 :
        print( "RP1210_ReadMessage returns %i" %nRetVal )

    tDisplayTime = ( time() - LastTimeDisplay )
    if tDisplayTime > 0.5 : 
        LastTimeDisplay = time()   
        print('Head: %0.1f Goal: %i Pow: %i L: %i R: %i Sats: %i Ang: %g Spd: %0.2f ' %(heading,goal,speed,leftMotor,rightMotor,gpsSats,gpsAngle,gpsSpeed), end='')
        if rightTurn:
            print('R Turn')
        elif leftTurn:
            print('L Turn')
        elif leftTurnAround:
            print('L Around')
        elif rightTurnAround:
            print('R Around')
        elif infinityTurn:
            print('Infinity')
        else:
            print('Normal')
########
    
    turnDeltaTime =  time()-lastTurnTime
    if turnDeltaTime>1:
        lastTurnTime=time()
        
        if degreeCount>90:
            degreeCount=0
            rightTurn=False
            leftTurn=False
        if turnAroundCount>180:
            turnAroundCount=0
            leftTurnAround=False
            rightTurnAround=False
        if infinityCount>540:
            infinityCount=0
            #infinityTurn=False
            
        if rightTurn:
            goal+=1
            degreeCount+=1
        elif leftTurn:
            goal-=1
            degreeCount+=1
        elif leftTurnAround:
            goal-=1
            turnAroundCount+=1
        elif rightTurnAround:
            goal+=1
            turnAroundCount+=1
        elif infinityTurn:        
            infinityCount+=1
            if infinityCount > 270:
                goal+=1
            else:
                goal-=1

    tDeltaTime = ( time() - LastTimeRequestsSent )

    if tDeltaTime > .05 :
        if speed > 100:
            speed = 100
        if speed < 0:
            speed = 0
        if goal > 360:
            goal -=360
        if goal < 0:
            goal +=360
            
        difference=goal-heading
        
        differenceList[i]=difference
        sum=0
        for d in differenceList:
            sum+=float(d)
        drift=sum/float(memorySize),
        i+= 1
        if i>=memorySize:
            i=0
        if difference <= -180:
            difference+=360
        elif difference > 180:
            difference-=360
        K=2
        I=0.4
        rightMotor=int(speed - K*difference - adjust - I*drift[0] + 100)
        leftMotor =int(speed + K*difference + adjust + I*drift[0] + 100)
        if rightMotor > 200:
            rightMotor=200
        if rightMotor < 0:
            rightMotor = 0
        if leftMotor > 200:
            leftMotor = 200
        if leftMotor < 0:
            leftMotor = 0
           
        SendCANSpeedMessage( nClientID,leftMotor,rightMotor )
        LastTimeRequestsSent = time() 

    
#-----------------------------------------------------------------------------------------------------
# Disconnect from the data bus.  Python will handle the cleanup and will call FreeLibrary().
#-----------------------------------------------------------------------------------------------------

nRetVal = RP1210_ClientDisconnect( c_short( nClientID ) )

if nRetVal == 0 :
   print("RP1210_ClientDisconnect - SUCCESS" )
else :
   print("RP1210_ClientDisconnect returns %i" %nRetVal )
#if
