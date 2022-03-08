from time import sleep
import serial
from geometry_msgs.msg import Twist # could use this to adapt to ROS
import sys
import rclpy
from rclpy.node import Node


# init serial comm
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()


class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Twist, 'linear_stuff', 1)

    def odom_publish(self, linear_l, linear_r): # function to publish
        msg = Twist()
        msg.linear.x = linear_l
        msg.linear.y = linear_r
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg) # this statement published the msg to the topic




def main(args=None):

    rclpy.init(args=args) # initialize node
    odom_publisher = OdomPublisher()
    while True:
        if ser.in_waiting > 0:

            line = ser.readline()
            #print(line)
            if line == b'\xff\n' or line == b'\xfe\n':
                continue
            serial_data = line.decode('utf-8').rstrip() #speeds is a string

            # split the string into a list ("," is where it splits)
            # leftCPS, rightCPS, linear_l, linear_r, linear, angular
            serial_data = serial_data.split(",")

            # convert each string element on the list into an int
            # serial_data = [int(i) for i in serial_data]
            # print(serial_data)

            leftCPS = serial_data[0]
            rightCPS = serial_data[1]
            linear_l = serial_data[2]
            linear_r = serial_data[3]
            linear = serial_data[4]
            angular = serial_data[5]

            print(leftCPS, ",", rightCPS, ",", linear_l, ",", linear_r, ",", linear, ",", angular)
            linear_l = float(linear_l)
            linear_r = float(linear_r)
            odom_publisher.odom_publish(linear_l, linear_r)

    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
