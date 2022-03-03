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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PhaseEnableRobot, LED



######### Combined the BetterRobot and MinimalSubscriber classes ######
class TheBot(Node): #phaseenablerobot doesn't need to be an argument because it is already being imported
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel', #may need to be changed to just '/cmd_vel'
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.robot = PhaseEnableRobot(left=(24,12), right=(25,13))
 
    def backward_left(self, speed = 0.99):
        self.robot.left_motor.backward(speed/2) #motor speed halved to keep robot moving backward AND turning left
        self.robot.right_motor.backward(speed)
        
    def backward_right(self, speed = 0.99):
        self.robot.left_motor.backward(speed)
        self.robot.right_motor.backward(speed/2) #motorspeed halved to keep the robot moving backward and turning right
    
    def forward_left(self, speed = 0.99):
        self.robot.left_motor.forward(speed/2) #motor speed halved to keep the robot moving forward AND turning left
        self.robot.right_motor.forward(speed)
        
    def forward_right(self, speed = 0.99):
        self.robot.left_motor.forward(speed)
        self.robot.right_motor.forward(speed/2) #motor speed halved to keep the robot moving forward AND turning right
        
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.linear.x)
        self.get_logger().info('I heard: "%s"' % msg.angular.z)
        #forward (I)
        if msg.linear.x > 0:
            self.robot.forward()
        #forward right (O)
        elif msg.linear.x > 0 and msg.angular.z > 0:
            self.forward_right()
        #forward left (U)
        elif msg.linear.x > 0 and msg.angular.z < 0:
            self.forward_left()
        #backward (<)
        elif msg.linear.x < 0:
            self.robot.backward()
        #backward right (>)
        elif msg.linear.x < 0 and msg.angular.z > 0:
            self.backward_right()
        #backward left (M)
        elif msg.linear.x < 0 and msg.angular.z < 0:
            self.backward_left()
        #left (J)
        elif msg.linear.x == 0 and msg.angular.z < 0:
            self.robot.left()
        #right (L)
        elif msg.linear.x == 0 and msg.angular.z > 0:
            self.robot.right()
        #robot stop (K)
        else:
            self.robot.stop()



def main(args=None):
     
    
    rclpy.init(args=args)

    print("I'm working")

    the_bot = TheBot() # creates object of class TheBot
    

    rclpy.spin(the_bot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    the_bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

