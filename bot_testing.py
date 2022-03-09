# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PhaseEnableRobot, LED
import time
import matplotlib.pyplot as plt
import numpy as np

class TheBot(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/linear_stuff',
            self.pid_control,
            10)
        self.subscription  # prevent unused variable warning
        self.robot = PhaseEnableRobot(right=(24,12), left=(25,13))
        self.linear_l = 0.0
        self.linear_r = 0.0
        self.desired_speed = 0.0
        self.p_left = 0.1
        self.p_right = 0.1 #This one works
        self.l_error = 0.0 # reference ang speed - actual ang speed
        self.r_error = 0.0
        self.l_pwm = 0.0
        self.r_pwm = 0.0
        self.start_time = 0.0
        self.left_speeds = []
        self.right_speeds = []
        self.time_data = []

    def pid_control(self, msg):
        print('Left Wheel Speed: ',  msg.linear.x)
        print('Right Wheel Speed: ', msg.linear.y)
        self.linear_l = msg.linear.x
        self.linear_r = msg.linear.y

        self.l_error = self.desired_speed - self.linear_l
        self.r_error = self.desired_speed - self.linear_r

        print("l_error", self.l_error)
        print("r_error", self.r_error)

        self.l_pwm = self.l_pwm + self.p_left*self.l_error
        self.r_pwm = self.r_pwm + self.p_right*self.r_error

        if (self.r_pwm > 0.999):
            self.r_pwm = 0.999
        if (self.l_pwm > 0.999):
            self.l_pwm = 0.999
        if (self.r_pwm < 0.1):
            self.r_pwm = 0.1
        if (self.l_pwm < 0.1):
            self.l_pwm = 0.1

        print("l_pwm ", self.l_pwm)
        print("r_pwm ", self.r_pwm)
        print(" ")
        self.robot.left_motor.forward(self.l_pwm)
        self.robot.right_motor.forward(self.r_pwm)

        self.time_data.append(time.time() - self.start_time)
        self.left_speeds.append(self.linear_l)
        self.right_speeds.append(self.linear_r)

def main(args=None):
    try:
        rclpy.init(args=args)

        print("I'm working")

        the_bot = TheBot() # creates object of class TheBot
        the_bot.desired_speed = 0.55 # m/s?

        the_bot.start_time = time.time()

        rclpy.spin(the_bot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    except KeyboardInterrupt:
        # open a data file for writing in same directory as the working program

        file = open('p_control_final.txt', 'w')
        for n in range(len(the_bot.time_data)):
            # write the data as comma delimited
            file.write(str(the_bot.time_data[n]) + ',' + str(the_bot.left_speeds[n]) + ',' + str(the_bot.right_speeds[n]) + '\n')
        # always close the file you are using!
        file.close()

        # make a plot
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)

        # make an xy scatter plot
        plt.scatter(the_bot.time_data,the_bot.left_speeds,color='red', label='Left Wheel')
        plt.scatter(the_bot.time_data,the_bot.right_speeds,color='blue', label='Right Wheel')

        # label the axes etc
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('P Control (p=0.1 for both)')
        plt.legend(loc = 'lower right') # legend location can be changed

        plt.savefig('P_control_0.11l_0.1r.png')

        the_bot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
