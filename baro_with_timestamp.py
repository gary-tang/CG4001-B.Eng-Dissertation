#!/usr/bin/env python
import time
import serial
from datetime import datetime

LOG_FILE_BAR = 'baro.csv'

# File more for opening the log file.  Mode 'ab' means append or add new lines
# to the end of the file rather than erasing it and starting over.  If you'd
# like to erase the file and start clean each time use the value 'wb' instead.
#LOG_MODE = 'ab'
LOG_MODE = 'wb'

#uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=30)
ser = serial.Serial('/dev/ttyACM0', 115200)

# Main loop just reads data from the GPS module and writes it back out to
# the output file while also printing to serial output.
with open (LOG_FILE_BAR, LOG_MODE) as outfile2:
    outfile2.write(bytes('time,pressure\n', 'utf-8'))
    while True:
        #use strip() if you want to remove additional newline when looking in console.
        #gps = uart.readline()
        baro = ser.readline().decode().rstrip()
        #print(str(baro, 'ascii'))
        
        timestamp = datetime.utcnow().strftime('%H%M%S.%f')[:-3]
        #print(datetime.utcnow().strftime('%H%M%S.%f')[:-3])
        
        reading = timestamp + ',' + baro
        print(reading)
        reading = bytes(reading + '\n', 'utf-8')
        
        outfile2.write(reading)
        outfile2.flush()

