#from matplotlib import pyplot
#import numpy as np

from LMD_firmata_addons import LMD_firmata_addons
from PyMata.pymata import PyMata
import time
import sys

print("hello")
board = LMD_firmata_addons('COM3')


def imu_data(x, y, z):
    print(" Receiving IMU Data")
    print('X = {0}'.format(x))
    print('Y = {0}'.format(y))
    print('Z = {0}'.format(z))


while True:
    board.read_imu(imu_data)
    time.sleep(1.0)

board.close()