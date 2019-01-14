#from matplotlib import pyplot
import numpy as np
from LMD_firmata_addons import LMD_firmata_addons
from PyMata.pymata import PyMata
import time
import sys

print("hello")
board = LMD_firmata_addons('COM3')

# Global Variables
a_compare = []
bno_ready = False                               # this is a flag that states when the IMU is out puting data
#v_compare = []

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
    global x_accel, y_accel, z_accel, i, a_compare, bno_ready
    print(" Receiving Acceleration Data in (m/s^2)")
    #print('x = {0}'.format(a_x))
    #print('y = {0}'.format(a_y))
    #print('z = {0}'.format(a_z))
    #print('t = {0}'.format(t))

    x_accel = np.vstack([x_accel, [t, a_x]])
    y_accel = np.vstack([y_accel, [t, a_y]])
    z_accel = np.vstack([z_accel, [t, a_z]])

    a_compare = [x_accel[i-1, :], x_accel[i, :], y_accel[i-1, :], y_accel[i, :], z_accel[i-1, :], z_accel[i, :]]
    bno_ready = True                            # if the code made it here, the imu is outputting data


def calc_vel():
    # from the gathered acceleration information, calculate the position of the IMU
    global x_vel, y_vel, z_vel, i, a_compare

    print('calculating the velocities')
    x_a0 = a_compare[0]                         # though gross, I would rather have this all be easy to follow
    x_a1 = a_compare[1]
    y_a0 = a_compare[2]
    y_a1 = a_compare[3]
    z_a0 = a_compare[4]
    z_a1 = a_compare[5]

    t_i = x_a1[0]                                # time used when logging the calculated values
    dt = x_a0[0] - x_a1[0]                 # find the change in time between the two data points
    da_x = x_a1[1] - x_a0[1]               # find the difference between the two acceleration values
    da_y = y_a1[1] - y_a0[1]
    da_z = z_a1[1] - z_a0[1]

    # calculate the velocity
    x_vel = np.vstack([x_vel, [t_i, da_x * dt]])     # calculate the x_velocity
    y_vel = np.vstack([y_vel, [t_i, da_y * dt]])
    z_vel = np.vstack([z_vel, [t_i, da_z * dt]])

    v_compare = [x_vel[i - 1, :], x_vel[i, :], y_vel[i - 1, :], y_vel[i, :], z_vel[i - 1, :], z_vel[i, :]]
    return v_compare


def calc_pose(v_compare):
    # from the gathered acceleration information, calculate the position of the IMU
    global x_pos, y_pos, z_pos, i

    print('calculating the position')
    x_v0 = v_compare[0]                         # though gross, I would rather have this all be easy to follow
    x_v1 = v_compare[1]
    y_v0 = v_compare[2]
    y_v1 = v_compare[3]
    z_v0 = v_compare[4]
    z_v1 = v_compare[5]

    t_i = x_v1[0]  # time used when logging the calculated values
    dt = x_v0[0] - x_v1[0]  # find the change in time between the two data points
    dv_x = x_v1[1] - x_v0[1]  # find the difference between the two acceleration values
    dv_y = y_v1[1] - y_v0[1]
    dv_z = z_v1[1] - z_v0[1]

    # calculate the position
    x_pos = np.vstack([x_pos, [t_i, dv_x * dt]])  # calculate the x_velocity
    y_pos = np.vstack([y_pos, [t_i, dv_y * dt]])
    z_pos = np.vstack([z_pos, [t_i, dv_z * dt]])

    print(' the current position is: ')
    print('x = {0}'.format(x_pos))
    print('y = {0}'.format(y_pos))
    print('z = {0}'.format(z_pos))

    i = i+1


while True:
    board.read_imu(imu_data)                                # gather orientation data from the imu
    board.read_accel(accel_data)                            # gather acceleration and time data from the imu
    if bno_ready:                                           # make sure the imu has been initialized before calculating
        vel_compare = calc_vel()                   # integrate the accelerometer info to calculate the velocity
        calc_pose(vel_compare)                                  # integrate the accelerometer into to find the position
    time.sleep(1.0)

board.close()