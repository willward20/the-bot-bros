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

#robot = PhaseEnableRobot(left=(24,12), right=(25,13)) #commented this line out to replace it with BetterRobot
robot = BetterRobot(left=(24,12), right=(25,13)) # edited verison of PhaseEnableRobot with new motor controls, comment out if this messes anything up

motor1=LED(23) #these two lines initialize the motors
motor2=LED(22) #

motor1.on() #These two lines activate the motor
motor2.on() #

######### PhaseEnableRobot edit to add forward left,right and backward left,right
### backward motor speed may need to be switched, needs to be tested
class BetterRobot(PhaseEnableRobot):
 
    def backward_left(self, speed = 1)
        self.left_motor.backward(speed/2) #motor speed halved to keep robot moving backward AND turning left
        self.right_motor.backward(speed)
        
    def backward_right(self, speed = 1)
        self.left_motor.backward(speed)
        self.right_motor.backward(speed/2) #motorspeed halved to keep the robot moving backward and turning right
    
    def forward_left(self, speed = 1):
        self.left_motor.forward(speed/2) #motor speed halved to keep the robot moving forward AND turning left
        self.right_motor.forward(speed)
        
    def forward_right(self, speed = 1):
        self.left_motor.forward(speed)
        self.right_motor.forward(speed/2)#motor speed halved to keep the robot moving forward AND turning right


#########

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel', #may need to be changed to just '/cmd_vel'
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        
################# Old callback function ##########################
#    def listener_callback(self, msg):
#        self.get_logger().info('I heard: "%s"' % msg.linear.x)
#        self.get_logger().info('I heard: "%s"' % msg.angular.z)
#        if msg.linear.x > 0:
#            robot.forward(0.35)
#        elif msg.linear.x < 0:
#            robot.backward(0.35)
#        else:
#            if msg.angular.z > 0:
#                robot.right(0.35)
#            else:
#                robot.left(0.35)
##################################################################                
                
  ###################  New callback function #####################         
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.linear.x)
        self.get_logger().info('I heard: "%s"' % msg.angular.z)
        #forward (I)
        if msg.linear.x > 0:
            robot.forward(0.35)
        #forward right (O)
        elif msg.linear.x > 0 and msg.angular.z > 0:
            robot.forward_right(0.35)
        #forward left (U)
        elif msg.linear.x > 0 and msg.angular.z < 0:
            robot.forward_left(0.35)
        #backward (<)
        elif msg.linear.x < 0:
            robot.backward(0.35)
        #backward right (>)
        elif msg.linear.x < 0 and msg.angular.z > 0:
            robot.backward_right(0.35)
        #backward left (M)
        elif msg.linear.x < 0 and msg.angular.z < 0:
            robot.backward_left(0.35)
        #robot stop
        else:
            robot.stop()
   #################################################################

def main(args=None):
     
    
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
