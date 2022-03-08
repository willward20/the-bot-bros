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
            self.drive_control,
            10)
        self.subscription  # prevent unused variable warning
        self.robot = PhaseEnableRobot(right=(24,12), left=(25,13))
        self.linear_l = 0.0
        self.linear_r = 0.0
        self.desired_speed = 0.5
    def drive_control(self, msg):
        self.robot.left_motor.forward(self.desired_speed)
        self.robot.right_motor.forward(self.desired_speed)
        time.sleep(2)



def main(args=None):

    rclpy.init(args=args)

    print("I'm working here")

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
