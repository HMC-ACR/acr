import serial
import time
import pynmea2

ser = serial.Serial("/dev/ttyACM0", 9600)

while True:
    msg = ser.readline().decode().strip('\r\n')
    if (msg[0:6] == '$GNGGA'):
        parsed_msg = pynmea2.parse(msg)
        print(parsed_msg.lat)
    time.sleep(1)
