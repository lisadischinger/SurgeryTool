#from matplotlib import pyplot
#import numpy as np

from LMD_firmata_addons import LMD_firmata_addons
from PyMata.pymata import PyMata
import time
import sys

print("hello")
board = LMD_firmata_addons('COM3')


def imu_data(omega, theta, zeta):
    print(" Receiving IMU Data")
    print('Omega = {0}'.format(omega))
    print('Theta = {0}'.format(theta))
    print('Zeta = {0}'.format(zeta))


def accel_data(x, y, z):
    print(" Receiving Acceleration Data in (m/s^2)")
    print('x = {0}'.format(x))
    print('y = {0}'.format(y))
    print('z = {0}'.format(z))


while True:
    board.read_imu(imu_data)
    board.read_accel(accel_data)
    time.sleep(1.0)

board.close()