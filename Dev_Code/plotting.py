# Plotting Speed Data

import matplotlib.pyplot as plt
import numpy as np

# define data arrays
time_data = []
linear_l = []
linear_r = []

# read in the data
lines = np.loadtxt('Data/.txt', delimiter=',') 
for line in lines:
    time_data.append(line[0]) # the first item in row is the time
    linear_l.append(line[1]) # the second item in row is left linear velocity
    linear_r.append(line[2]) # the third item in the row is the right linear velocity


# make a plot
fig = plt.figure()
ax = fig.add_subplot(1,1,1)

# make an xy scatter plot
plt.scatter(time_data,linear_l,color='red', label='Left Wheel')
plt.scatter(time_data,linear_r,color='blue', label='Right Wheel')

# label the axes etc
ax.set_xlabel('Time (s)')
ax.set_ylabel('Velocity (m/s)')
ax.set_title('')
plt.legend(loc = 'lower right') # legend location can be changed

plt.savefig('Plots/.png')