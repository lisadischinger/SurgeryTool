import sys, csv
from pyfirmata import Arduino, util
#from matplotlib import pyplot
#import numpy as np
import time

print("hello")
board = Arduino('COM3')                         # Specify the serial port found through Device Manager
board.digital[13].write(1)                      #set pin 13 high for the light

# designation of pins
pwm_pin_out = board.get_pin('d:3:p')               # pin for pwm output( digital, pin 3, and it is an pwm signal)
pwm_pin_in = board.get_pin('a:0:i')                 # pin for reading the PWM created

# pin setup
it = util.Iterator(board)
it.start()
board.analog[0].enable_reporting()

pwm_range = np.arange(0, 1, 0.05)              # create an array to brighten and dim light
t_inc = 0.005                                   # increment of time to read pwm signal
y = [0] * int(20/t_inc)
t_i = [0] * int(20/t_inc)
t0 = time.time()                                # keep track of initial time for elapsed time calc.
i = 1                                           # i is for incrementing the pwm signal up

for j in range(1, len(t_i)-1):
    pwm_pin_out.write(pwm_range[i])              # pwm signal is between 0 and .6
    t_i[j] = time.time() - t0                    # elapsed time
    y[j] = pwm_pin_in.read()
    if t_i[j] - int(t_i[j]) == 0:                # only increase the pwm signal every two seconds
        i = i + 1


print(t_i)
print(y)
pyplot.plot(t_i, y)
pyplot.show()
print("I'm done")
