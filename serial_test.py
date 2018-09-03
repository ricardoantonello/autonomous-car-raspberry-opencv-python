#!/usr/bin/python3
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600)

# read from Arduino
#i = ser.read()
#print ("Read input " + i.decode("utf-8") + " from Arduino")
#print ("Read input " + str(i) + " from Arduino\n")

#ser.write("b".encode())

for i in range(10):
    print('Teste', i)
    ser.write(str(i).encode())
    ser.flush()
    time.sleep(3)
#ser.close()
