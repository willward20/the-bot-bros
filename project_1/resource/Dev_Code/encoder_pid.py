from time import sleep
from gpiozero import LED, PhaseEnableMotor
import serial
import sys
import numpy as np

# initialize constants
gear_ratio = 70
CPR = 64 * gear_ratio
wheel_radius = 4.215/100 # 4.215 cm
delay_time = 0.01 # 10 milliseconds

# init motor
LMotor = PhaseEnableMotor(24,12)
RMotor = PhaseEnableMotor(25,13)

# init serial comm
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()

# PID Constants
duty_cycle_L = 0.0
duty_cycle_R = 0.0
Kp = 0.5
reference_speed = 3.14159 # rad/s


try:
    while True:
        LMotor.forward(duty_cycle_L)
        RMotor.forward(duty_cycle_R)
        sleep(0.1)

        if ser.in_waiting > 0:

            line = ser.readline()
            counts = line.decode('utf-8').rstrip() #speeds is a string

            # split the string into a list ("," is where it splits)
            counts = counts.split(",")

            # convert each string element on the list into an int
            counts = [int(i) for i in counts]
            print('Counts (Left, Right): ', counts)

            # Calculate Wheel Angular Speed
            ang_speed_left = abs(counts[0]*2*3.14159/CPR/delay_time)
            #print('Left Wheel Ang Speed: ', ang_speed_left)
            ang_speed_right = abs(counts[1]*2*3.14159/CPR/delay_time)
            #print('Right Wheel Ang Speed: ', ang_speed_right)

            # Calculate Wheel Linear Speed
            linear_left = wheel_radius * ang_speed_left
            print('Left Linear Speed: ', linear_left)
            linear_right = wheel_radius * ang_speed_right
            print('Right Linear Speed: ', linear_right)

            #Print Odometery Information
            #avg_robot_speed = (linear_left + linear_right) / 2
            #print("Robot Linear Speed (m/s): ", avg_robot_speed)
            #print("")



            # Proportional Control
            error_L = reference_speed - linear_left
            error_R = reference_speed - linear_right
            proportional_L = Kp*error_L
            proportional_R = Kp*error_R
            duty_cycle_L = duty_cycle_L + proportional_L
            duty_cycle_R = duty_cycle_R + proportional_R
            if duty_cycle_L >= 1:
                duty_cycle_L = 1
            elif duty_cycle_L <= 0:
                duty_cycle_L = 0
            if duty_cycle_R >= 1:
                duty_cycle_R = 1
            elif duty_cycle_R <= 0:
                duty_cycle_R = 0


except KeyboardInterrupt:
     print("Interrupted")

finally:
    LMotor.stop()
    RMotor.stop()
