# Plotting Speed Data

import matplotlib.pyplot as plt
import numpy as np

# define data arrays
time_data_p_001 = []
p_001_l = []
p_001_r = []

time_data_p_004 = []
p_004_l = []
p_004_r = []

time_data_p_007 = []
p_007_l = []
p_007_r = []

time_data_p_01 = []
p_01_l = []
p_01_r = []

time_data_p_013 = []
p_013_l = []
p_013_r = []

time_data_p_016 = []
p_016_l = []
p_016_r = []

time_data_p_019 = []
p_019_l = []
p_019_r = []

time_data_p_025 = []
p_025_l = []
p_025_r = []

# read in the data
lines = np.loadtxt('Data/p_0.01.txt', delimiter=',') 
for line in lines:
    time_data_p_001.append(line[0]) # the first item in row is the time
    p_001_l.append(line[1]) # the second item in row is left linear velocity
    p_001_r.append(line[2]) # the third item in the row is the right linear velocity

# read in the data
lines = np.loadtxt('Data/p_0.04.txt', delimiter=',') 
for line in lines:
    time_data_p_004.append(line[0]) # the first item in row is the time
    p_004_l.append(line[1]) # the second item in row is left linear velocity
    p_004_r.append(line[2]) # the third item in the row is the right linear velocity

# read in the data
lines = np.loadtxt('Data/p_0.07.txt', delimiter=',') 
for line in lines:
    time_data_p_007.append(line[0]) # the first item in row is the time
    p_007_l.append(line[1]) # the second item in row is left linear velocity
    p_007_r.append(line[2]) # the third item in the row is the right linear velocity

# read in the data
lines = np.loadtxt('Data/p_0.1.txt', delimiter=',') 
for line in lines:
    time_data_p_01.append(line[0]) # the first item in row is the time
    p_01_l.append(line[1]) # the second item in row is left linear velocity
    p_01_r.append(line[2]) # the third item in the row is the right linear velocity

# read in the data
lines = np.loadtxt('Data/p_0.13.txt', delimiter=',') 
for line in lines:
    time_data_p_013.append(line[0]) # the first item in row is the time
    p_013_l.append(line[1]) # the second item in row is left linear velocity
    p_013_r.append(line[2]) # the third item in the row is the right linear velocity

# read in the data
lines = np.loadtxt('Data/p_0.16.txt', delimiter=',') 
for line in lines:
    time_data_p_016.append(line[0]) # the first item in row is the time
    p_016_l.append(line[1]) # the second item in row is left linear velocity
    p_016_r.append(line[2]) # the third item in the row is the right linear velocity

# read in the data
lines = np.loadtxt('Data/p_0.19.txt', delimiter=',') 
for line in lines:
    time_data_p_019.append(line[0]) # the first item in row is the time
    p_019_l.append(line[1]) # the second item in row is left linear velocity
    p_019_r.append(line[2]) # the third item in the row is the right linear velocity

# read in the data
lines = np.loadtxt('Data/p_0.25.txt', delimiter=',') 
for line in lines:
    time_data_p_025.append(line[0]) # the first item in row is the time
    p_025_l.append(line[1]) # the second item in row is left linear velocity
    p_025_r.append(line[2]) # the third item in the row is the right linear velocity

# trim to 3.5 seconds
n = -1
while time_data_p_007[n] > 3:
    del time_data_p_007[n]
    del p_007_l[n]
n = -1
while time_data_p_01[n] > 3:
    del time_data_p_01[n]
    del p_01_l[n]
n = -1
while time_data_p_013[n] > 3:
    del time_data_p_013[n]
    del p_013_l[n]
n = -1
while time_data_p_016[n] > 3:
    del time_data_p_016[n]
    del p_016_l[n]
n = -1
while time_data_p_019[n] > 3:
    del time_data_p_019[n]
    del p_019_l[n]
n = -1
while time_data_p_025[n] > 3:
    del time_data_p_025[n]
    del p_025_l[n]

# make a plot
fig = plt.figure()
ax = fig.add_subplot(1,1,1)

# add a step function
x = [0, 3] 
y = [0, 0.55]
plt.step(x, y, color='orange', linestyle='dashed', label='Reference Speed 0.55 m/s')

# make an xy scatter plot
#plt.step(time_data_p_001,p_001_l,color='red', label='p=0.01')
#plt.step(time_data_p_004,p_004_l,color='blue', label='p=0.04')
#plt.step(time_data_p_007,p_007_l,color='green', label='p=0.07')
plt.step(time_data_p_01,p_01_l,color='purple', label='p=0.1')
#plt.step(time_data_p_013,p_013_l,color='black', label='Left p=0.13')
plt.step(time_data_p_016,p_016_l,color='red', label='Left p=0.16')
#plt.step(time_data_p_019,p_019_l,color='blue', label='Left p=0.19')
plt.step(time_data_p_025,p_025_l,color='green', label='Left p=0.25')
#plt.step(time_data,linear_r,color='blue', label='Right Wheel')

# label the axes etc
ax.set_xlabel('Time (s)')
ax.set_ylabel('Velocity (m/s)')
ax.set_title('Comparing P Values for Left Wheel (0.1 to 0.25)')
plt.legend(loc = 'lower right') # legend location can be changed

plt.savefig('Plots/comapre_p_2_chopped.png')
plt.show()