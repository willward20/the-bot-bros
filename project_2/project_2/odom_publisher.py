import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from gpiozero import PhaseEnableRobot, LED


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        # init tf broadcaster
        self.odom_broadcaster = TransformBroadcaster(self)
        # init robot velocity subscriber
        self.vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.vel_sub_cb,
            1
        )
        self.vel_sub  # prevent unused variable warning
        # init odom topic publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 1)
        # init a timer for publishing tfs and topics
        self.odom_pub_timer = self.create_timer(0.1, self.odom_pub_timer_cb)
        # variables
        self.x = 0.
        self.y = 0.
        self.th = 0.
        self.lin_x = 0.
        self.ang_z = 0.
        self.cur_time = self.get_clock().now()
        self.pre_time = self.get_clock().now()
        # self.i = 0

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


    def vel_sub_cb(self, msg):
        """
        callback function for subsriber
        Input:
            msg: (target) robot velocity
        """
        self.lin_x = msg.linear.x
        self.ang_z = msg.angular.z

        #forward motions
        if msg.linear.x > 0:
            #forward right (O)
            if msg.angular.z > 0:
                self.forward_right()
            #forward left (U)
            elif msg.angular.z < 0:
                self.forward_left()
            #straight forward(I)
            else:
                self.robot.forward()
        #backward motions
        elif msg.linear.x < 0:
            #backward right (>)
            if msg.angular.z > 0:
                self.backward_right()
            #backward left (M)
            elif msg.angular.z < 0:
                self.backward_left()
            #straight backward (>)
            else:
                self.robot.backward()
        #left (J)
        elif msg.linear.x == 0 and msg.angular.z < 0:
            self.robot.left()
        #right (L)
        elif msg.linear.x == 0 and msg.angular.z > 0:
            self.robot.right()
        #robot stop (K)
        else:
            self.robot.stop()

    def odom_pub_timer_cb(self):
        """
        timer callback function for periodically executed jobs
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
        self.get_logger().debug(f'Publishing: {msg}')
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


if __name__ == '__main__':
    main()
