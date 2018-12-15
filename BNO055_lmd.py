#from matplotlib import pyplot
import numpy as np

from LMD_firmata_addons import LMD_firmata_addons
from PyMata.pymata import PyMata
import time
import sys

print("hello")
board = LMD_firmata_addons('COM3')

# Global Variables
x_accel = [0, 0]                                # create empty lists so that we can gather information over time;
y_accel = [0, 0]                                    # initial values have been put in
z_accel = [0, 0]
x_vel = [0, 0]
y_vel = [0, 0]
z_vel = [0, 0]
x_pos = [0, 0]
y_pos = [0, 0]
z_pos = [0, 0]

x_a0 = 0
x_a1 = 0
y_a0 = 0
y_a1 = 0
z_a0 = 0
z_a1 = 0

i = 1                                       # counter used to append to the lists and read from the lists
                                                # 1 is already accounted for since there are the set initial conditions


def imu_data(omega, theta, zeta):
    print(" Receiving orinetation Data [deg]")
    #print('Omega = {0}'.format(omega))
    #print('Theta = {0}'.format(theta))
    #print('Zeta = {0}'.format(zeta))


def accel_data(a_x, a_y, a_z, t):
    global x_accel, y_accel, z_accel, i
    print(" Receiving Acceleration Data in (m/s^2)")
    print('x = {0}'.format(a_x))
    print('y = {0}'.format(a_y))
    print('z = {0}'.format(a_z))
    print('t = {0}'.format(t))

    x_accel = np.vstack([x_accel, [t, a_x]])
    y_accel = np.vstack([y_accel, [t, a_y]])
    z_accel = np.vstack([z_accel, [t, a_z]])

    a_compare = [x_accel[i-1, :], x_accel[i, :], y_accel[i-1, :], y_accel[i, :], z_accel[i-1, :], z_accel[i, :]]
    return a_compare               # this will return the values that will be compared for the calcs


def calc_vel(a_compare):
    # from the gathered acceleration information, calculate the position of the IMU
    global x_vel, y_vel, z_vel, i

    x_a0 = a_compare[1]                         # though gross, I would rather have this all be easy to follow
    x_a1 = a_compare[2]
    y_a0 = a_compare[3]
    y_a1 = a_compare[4]
    z_a0 = a_compare[5]
    z_a1 = a_compare[6]

    t_i = x_a1[1, 1]                             # time used when logging the calculated values
    dt = x_a0[1, 1] - x_a1[1, 1]                 # find the change in time between the two data points
    da_x = x_a1[1, 2] - x_a0[1, 2]               # find the difference between the two acceleration values
    da_y = y_a1[1, 2] - y_a0[1, 2]
    da_z = z_a1[1, 2] - z_a0[1, 2]

    # calculate the velocity
    x_vel = np.column_stack((t_i, da_x * dt))     # calculate the x_velocity
    y_vel = np.column_stack((t_i, da_y * dt))
    z_vel = np.column_stack((t_i, da_z * dt))

    v_compare = [x_vel[i - 1, :], x_vel[i, :], y_vel[i - 1, :], y_vel[i, :], z_vel[i - 1, :], z_vel[i, :]]
    return v_compare


def calc_pose(v_compare):
    # from the gathered acceleration information, calculate the position of the IMU
    global x_pos, y_pos, z_pos, i

    x_v0 = v_compare[1]                         # though gross, I would rather have this all be easy to follow
    x_v1 = v_compare[2]
    y_v0 = v_compare[3]
    y_v1 = v_compare[4]
    z_v0 = v_compare[5]
    z_v1 = v_compare[6]

    t_i = x_v1[1, 1]  # time used when logging the calculated values
    dt = x_v0[1, 1] - x_v1[1, 1]  # find the change in time between the two data points
    dv_x = x_v1[1, 2] - x_v0[1, 2]  # find the difference between the two acceleration values
    dv_y = y_v1[1, 2] - y_v0[1, 2]
    dv_z = z_v1[1, 2] - z_v0[1, 2]

    # calculate the velocity
    x_pos = np.column_stack((t_i, dv_x * dt))  # calculate the x_velocity
    y_pos = np.column_stack((t_i, dv_y * dt))
    z_pos = np.column_stack((t_i, dv_z * dt))

    print(' the current position is: ')
    print('x = {0}'.format(x_pos))
    print('y = {0}'.format(y_pos))
    print('z = {0}'.format(z_pos))

    i = i+1

while True:
    board.read_imu(imu_data)                                # gather orientation data from the imu
    accel_compare = board.read_accel(accel_data)            # gather acceleration and time data from the imu
    print('a_compare = {0}'.format(accel_compare))
    #vel_compare = calc_vel(accel_compare)                   # integrate the accelerometer info to calculate the velocity
    #calc_pose(vel_compare)                                  # integrate the accelerometer into to find the position
    time.sleep(1.0)

board.close()