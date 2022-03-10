# used to integrate velocity data and find distance traveled

import time
import numpy as np


time_data = []
velocity_data = []

# read in the data
lines_1 = np.loadtxt('Data/p_control_distance_test.txt', delimiter=',') # file name for 1 k OHMS data
for line in lines_1:
    time_data.append(line[0]) # the first item in row is the time
    velocity_data.append(line[1]) # the second item in row is the voltage

# calculate distance traveled
d_l = 0.0
for n in range (1, len(time_data)):
    d_l = d_l + (velocity_data[n-1] * (time_data[n]-time_data[n-1]))
print("under estimate: ", d_l)

d_r = 0.0
for n in range(1, len(time_data)):
    d_r = d_r + (velocity_data[n]* (time_data[n]-time_data[n-1]))
print("over estimate: ", d_r)

d_av = (d_l + d_r) / 2

print("average distance: ", d_av)

