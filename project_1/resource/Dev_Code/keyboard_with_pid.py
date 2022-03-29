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

"""
NOTE: This program is currently under construction. I want to combine PID 
      control and keyboard input. Still a work in progress.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PhaseEnableRobot, LED
import time


class TheBot(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/linear_stuff',
            self.pid_control,
            10)
        self.subscription  # prevent unused variable warning

        self.subscription2 = self.create_subscription(
            Twist,
            '/cmd_vel', #may need to be changed to just '/cmd_vel'
            self.get_keyboard,
            10)
        self.subscription2  # prevent unused variable warning


        self.robot = PhaseEnableRobot(right=(24,12), left=(25,13))
        self.linear_l = 0.0
        self.linear_r = 0.0
        self.desired_speed = 0.0
        self.p = 0.1 #proportionality constant (for P)
        self.l_error = 0.0 # reference ang speed - actual ang speed
        self.r_error = 0.0
        self.l_pwm = 0.0
        self.r_pwm = 0.0

    def pid_control(self, msg):
        print('Left Wheel Speed: ',  msg.linear.x)
        print('Right Wheel Speed: ', msg.linear.y)
        self.linear_l = msg.linear.x
        self.linear_r = msg.linear.y

        self.l_error = self.desired_speed - self.linear_l
        self.r_error = self.desired_speed - self.linear_r

        print("l_error", self.l_error)
        print("r_error", self.r_error)

        self.l_pwm = self.l_pwm + self.p*self.l_error
        self.r_pwm = self.r_pwm + self.p*self.r_error

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


    def get_keyboard(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.linear.x)
        self.get_logger().info('I heard: "%s"' % msg.angular.z)
        # finish this later!! Find a way to incorporate keyboard input into
        # the pid. Need to seperate into left and right wheels for each command

def main(args=None):

    rclpy.init(args=args)

    print("I'm working")

    the_bot = TheBot() # creates object of class TheBot
    the_bot.desired_speed = 0.6 # m/s?

    rclpy.spin(the_bot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    the_bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
