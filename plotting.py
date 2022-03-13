# Plotting Speed Data

import matplotlib.pyplot as plt
import numpy as np

# define data arrays
time_data = []
linear_l = []
linear_r = []

# read in the data
lines = np.loadtxt('.txt', delimiter=',') 
for line in lines:
    time_data.append(line[0]) # the first item in row is the time
    linear_l.append(line[1]) # the second item in row is left linear velocity
    linear_r.append(line[2]) # the third item in the row is the right linear velocity

# trim to 3.5 seconds
n = -1
while time_data[n] > 2.5:
    del time_data[n]
    del linear_l[n]
    del linear_r[n]

# make a plot
fig = plt.figure()
ax = fig.add_subplot(1,1,1)

# add a step function
x = [0, 2.5] 
y = [0, 0.57]
plt.step(x, y, color='orange', linestyle='dashed', label='Reference Speed 0.57 m/s')

# make an xy scatter plot
plt.step(time_data,linear_l,color='red', label='Left Wheel')
plt.step(time_data,linear_r,color='blue', label='Right Wheel')

# label the axes etc
ax.set_xlabel('Time (s)')
ax.set_ylabel('Velocity (m/s)')
ax.set_title('')
plt.legend(loc = 'lower right') # legend location can be changed

plt.savefig('.png')
plt.show()