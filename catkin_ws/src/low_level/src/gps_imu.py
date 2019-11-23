import serial
import time
import pynmea2
import board
import busio
import adafruit_bno055
import rospy
import rospkg
from geometry_msgs.msg import Pose
from nag_msgs.msgs import Odometry

# i2c device must be on Jetson's i2c bus 1 

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

# set up serial connection to GPS
ser = serial.Serial("/dev/ttyACM0", 9600)

while True:

    # Read GPS
    msg = ser.readline().decode().strip('\r\n')
    if (msg[0:6] == '$GNGGA'):
        parsed_msg = pynmea2.parse(msg)
        print()
        print('Latitude: {}'.format(parsed_msg.lat))
        print('Longitude: {}'.format(parsed_msg.lon))

    # Read IMU
    print('Accelerometer (m/s^2): {}'.format(sensor.acceleration))
    print('Euler angle: {}'.format(sensor.euler))
    print('Linear acceleration (m/s^2): {}'.format(sensor.linear_acceleration))
    print()
 
    time.sleep(1)

class GPSIMU():

    def __init__(self):

if __name__ == '__main__':
    try:
        gpsimu = GPSIMU()
    except rospy.ROSInterruptException:
        pass
