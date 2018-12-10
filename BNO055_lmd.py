from pyfirmata import Arduino, util
#from matplotlib import pyplot
#import numpy as np

import time
import sys

print("hello")
board = Arduino('COM3')                         # Specify the serial port found through Device Manager


def imu_data(x, y, z):
    print(" Receiving IMU Data")
    print('X = {0}'.format(x))
    print('Y = {0}'.format(y))
    print('Z = {0}'.format(z))


while True:
    board.read_imu(imu_data)
    time.sleep(1.0)

board.close()