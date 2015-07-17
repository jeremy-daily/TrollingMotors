#!/env/Python
#Serial Data Logger
import serial
import os
import sys
import time

ser = serial.Serial(port="COM4",baudrate=115200)
time.sleep(2);
file = open("SerialDataLog.txt",'w')
file.write(time.asctime())
file.write('\n')
file.close()
while True:
	data=ser.readline()
	file = open("SerialDataLog.txt",'ab')
	file.write(data)
	print(str(data[:-1],'ASCII'))
	file.close()
	#time.sleep(.200);
	if 	os.path.getsize("SerialDataLog.txt") > 20e6:
		sys.exit()