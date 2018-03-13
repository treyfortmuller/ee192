# EECS192 Spring 2018 Track Finding from 1D line sensor data
# testing matplot library as we'll use it in HW1

import numpy as np
#import scipy as sp
import matplotlib.pyplot as plt
# import scipy.ndimage as ndi  # useful for 1d filtering functions
plt.close("all")   # try to close all open figs


time = [1, 2, 3, 4, 5]
vel = [2, 4, 6, 8, 10]

fig = plt.figure(figsize = (8, 4))
fig.set_size_inches(13, 4)
fig.suptitle("velocity vs. time")   
plt.xlabel('time [s]')
plt.ylabel('velocity [m/s]')  
plt.plot(time,vel)

plt.show() 