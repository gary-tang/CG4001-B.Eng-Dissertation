#!/usr/bin/env python
import time
import serial

ser = serial.Serial('/dev/ttyACM0', 115200)


while 1:
 x=ser.readline().decode().strip()
 print (x)