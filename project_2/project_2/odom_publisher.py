import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations
from tf2_ros import TransformBroadcaster
import serial
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from gpiozero import PhaseEnableRobot, LED
import sys
import math
import time

# init serial comm
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")
        # init robot
        self.robot = PhaseEnableRobot(right=(24, 12), left=(25, 13))
        # init tf broadcaster
        self.odom_broadcaster = TransformBroadcaster(self)
        # init robot velocity subscriber
        self.vel_sub = self.create_subscription(Twist, "/cmd_vel", self.vel_sub_cb, 1)
        self.vel_sub  # prevent unused variable warning
        # init odom topic publisher
        self.odom_pub = self.create_publisher(Odometry, "/odom", 1)
        # init a timer for publishing tfs and topics
        self.odom_pub_timer = self.create_timer(0.01, self.odom_pub_timer_cb)
        # timer callback function for PID control
        self.pid_timer = self.create_timer(0.01, self.pid_timer_cb)


        # variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.lin_x = 0.0 # /cmd_vel command
        self.ang_z = 0.0 # /cmd_vel command
        self.cur_time = self.get_clock().now()
        self.pre_time = self.get_clock().now()
        # self.i = 0
        self.L = 0.42 # distance between the wheels

        # PID stuff
        self.max = 0.7 # max wheel speed for mapping
        self.min = 0 # minimum wheel speed
        self.linear_l = 0.0
        self.linear_r = 0.0
        self.p = 0.1
        self.l_error = 0.0 # reference ang speed - actual ang speed
        self.r_error = 0.0
        self.l_pwm = 0.0
        self.r_pwm = 0.0
        self.start_time = 0.0
        self.left_speeds = []
        self.right_speeds = []
        self.time_data = []
        self.l_state = 1 #1 if forward, -1 if backward
        self.r_state = 1 #1 if forward, -1 if backward


    def vel_sub_cb(self, msg):
        # get the target speeds from /cmd_vel
        self.lin_x = msg.linear.x
        self.ang_z = msg.angular.z



    def pid_timer_cb(self):
        # all of PID control
        # convert linear and angular to left and right wheel speeds
        v_l = self.lin_x - (self.ang_z * self.L / 2) # target left wheel lin speed
        v_r = self.lin_x + (self.ang_z * self.L / 2) # target right wheel lin speed
        print(" ")
        print("v_l: ",  v_l)
        print("v_r: ", v_r)
        print("real lin_l: ", self.linear_l)
        print("real lin_r: ", self.linear_r)

        self.linear_l = self.linear_l * self.l_state
        self.linear_r = self.linear_r * self.r_state

        # compare to current LR wheel speeds
        # incrment or decrement the speeds
        self.l_error = v_l - self.linear_l
        self.r_error = v_r - self.linear_r

        print("l_error: ", self.l_error)
        print("r_error: ", self.r_error)

        print("l_pwm previous: ", self.l_pwm)         
        print("r_pwm previous: ", self.r_pwm)

        if (v_l > 0):
            self.l_pwm = self.l_pwm + self.p*self.l_error
        elif (v_l < 0):
            self.l_pwm = self.l_pwm - self.p*self.l_error
        else:
            if self.l_state == 1:
                self.l_pwm = self.l_pwm + self.p*self.l_error
            else:
                self.l_pwm = self.l_pwm - self.p*self.l_error

        if (v_r > 0):
            self.r_pwm = self.r_pwm + self.p*self.r_error
        elif (v_r < 0):
            self.r_pwm = self.r_pwm - self.p*self.r_error
        else:
            if self.r_state == 1:
                self.r_pwm = self.r_pwm + self.p*self.r_error
            else:
                self.r_pwm = self.r_pwm - self.p*self.r_error

        self.l_pwm = abs(self.l_pwm)
        self.r_pwm = abs(self.r_pwm)
    
        print("l_pwm new: ", self.l_pwm)         
        print("r_pwm new: ", self.r_pwm)



        if (self.r_pwm > self.max):
            self.r_pwm = self.max
        if (self.l_pwm > self.max):
            self.l_pwm = self.max
#        if (self.r_pwm < self.min):
 #           self.r_pwm = self.min
  #      if (self.l_pwm < self.min):
   #         self.l_pwm = self.min
  #      print("l_pwm: ", self.l_pwm)         
   #     print("r_pwm: ", self.r_pwm)
        # print(" ")
        if (v_l > 0):
            self.robot.left_motor.forward(self.l_pwm)
            self.l_state = 1
        elif (v_l < 0):
            self.robot.left_motor.backward(self.l_pwm)
            self.l_state = -1
            print("going backward and was told negative")
        else:
            if self.l_state == 1:
                self.robot.left_motor.forward(self.l_pwm)
            else:
                self.robot.left_motor.backward(self.l_pwm)
        print("l_state: ", self.l_state)

        if (v_r > 0):
            self.robot.right_motor.forward(self.r_pwm)
            self.r_state = 1
        elif (v_r < 0):
            self.robot.right_motor.backward(self.r_pwm)
            self.r_state = -1
        else:
            if self.r_state == 1:
                self.robot.right_motor.forward(self.r_pwm)
            else:
                self.robot.right_motor.backward(self.r_pwm)
        print("r_state: ", self.r_state)



    def odom_pub_timer_cb(self):
        """
        timer callback function for periodically executed jobs
        """


        # get the encoder readings
        if ser.in_waiting > 0:
            line = ser.readline()
            #print(line)
            if line == b'\xff\n' or line == b'\xfe\n':
                dummy = 0 # for now
            serial_data = line.decode('utf-8').rstrip() #speeds is a string

            # split the string into a list ("," is where it splits)
            # leftCPS, rightCPS, linear_l, linear_r, linear, angular
            serial_data = serial_data.split(",")

            # convert each string element on the list into an int
            # serial_data = [int(i) for i in serial_data]
            # print(serial_data)

            #leftCPS = serial_data[0]
            #rightCPS = serial_data[1]
            self.linear_l = serial_data[2]
            self.linear_r = serial_data[3]
            #linear = serial_data[4]
            #angular = serial_data[5]
            self.linear_l = float(self.linear_l)
            self.linear_r = float(self.linear_r)

        """
        Note: update the following code to use real encoder data for odom!!
        Note: update the following code to use real encoder data for odom!!
        Note: update the following code to use real encoder data for odom!!
        Note: update the following code to use real encoder data for odom!!
        Note: update the following code to use real encoder data for odom!!
        """

        # update pose
        self.cur_time = self.get_clock().now()
        dt = (self.cur_time - self.pre_time).nanoseconds * 1e-9  # convert to seconds
        # print(dt)
        delta_x = self.lin_x * np.cos(self.th) * dt
        delta_y = self.lin_x * np.sin(self.th) * dt
        delta_th = self.ang_z * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        q = tf_transformations.quaternion_about_axis(self.th, (0, 0, 1))
        # prepare odom transform
        trans = TransformStamped()
        trans.header.stamp = self.cur_time.to_msg()
        trans.header.frame_id = "odom"
        trans.child_frame_id = "base_link"
        trans.transform.translation.x = self.x
        trans.transform.translation.y = self.y
        trans.transform.translation.z = 0.0
        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]
        self.odom_broadcaster.sendTransform(trans)
        # prepare odom topic
        msg = Odometry()
        msg.header.stamp = self.cur_time.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = self.lin_x
        msg.twist.twist.angular.z = self.ang_z
        self.odom_pub.publish(msg)
        self.get_logger().debug(f"Publishing: {msg}")
        self.pre_time = self.cur_time
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdometryPublisher()

    rclpy.spin(odom_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
