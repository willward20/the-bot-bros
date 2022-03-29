from time import sleep
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry # is this right?
import sys
import rclpy
from rclpy.node import Node


# init serial comm
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()


class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Twist, '/linear_stuff', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel', 
            self.listener_callback, # drive robot
            10)
        self.subscription  # prevent unused variable warning
        self.robot = PhaseEnableRobot(left=(24,12), right=(25,13))

        # define variables for storing direction and speed

    def timer_callback(self):
        if ser.in_waiting > 0:

            line = ser.readline()
            #print(line)
            if line == b'\xff\n' or line == b'\xfe\n':
                continue
            serial_data = line.decode('utf-8').rstrip() #speeds is a string

            # leftCPS, rightCPS, linear_l, linear_r, linear, angular
            serial_data = serial_data.split(",")

            leftCPS = serial_data[0]
            rightCPS = serial_data[1]
            linear_l = serial_data[2]
            linear_r = serial_data[3]
            linear = serial_data[4]
            angular = serial_data[5]

#            print(leftCPS, ",", rightCPS, ",", linear_l, ",", linear_r, ",", linear, ",", angular)
            linear_l = float(linear_l)
            linear_r = float(linear_r)

        # save the direction and speed to object variables

        msg = 
        msg.data = 
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1



    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.linear.x)
        self.get_logger().info('I heard: "%s"' % msg.angular.z)
        
        
        # access current direciton and speed from object variables

        # compare current wheel speeds to command wheel speeds 
        # pid wheel speeds 



def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdomPublisher()

    rclpy.spin(odom_publisher)


    # Destroy the node explicitly
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()