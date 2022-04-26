import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import board
import adafruit_mpu6050


class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.imu = adafruit_mpu6050.MPU6050(i2c)
        self.imu_publisher_ = self.create_publisher(Imu, 'imu', 2)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.orientation_covariance = [-1.] * 9
        msg.angular_velocity.x = self.imu.gyro[0]
        msg.angular_velocity.y = self.imu.gyro[1]
        msg.angular_velocity.z = self.imu.gyro[2]
        msg.linear_acceleration.x = self.imu.acceleration[0]
        msg.linear_acceleration.y = self.imu.acceleration[1]
        msg.linear_acceleration.z = self.imu.acceleration[2]
        self.get_logger().debug(f'{msg}')
        self.imu_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    imu_pub = ImuPublisher()

    rclpy.spin(imu_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
