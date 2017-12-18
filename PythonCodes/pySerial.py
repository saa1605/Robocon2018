import serial

ser = serial.Serial("COM3", 9600)

while(1):
    ser.write(b's')
##ser.close()
