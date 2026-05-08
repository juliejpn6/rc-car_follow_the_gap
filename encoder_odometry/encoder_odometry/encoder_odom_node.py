import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import pigpio
import math
import time


class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_odom_node')

        # --- Parameters ---
        self.declare_parameter('encoder_a_pin', 22)
        self.declare_parameter('encoder_b_pin', 27)
        self.declare_parameter('pulses_per_rev', 36)
        self.declare_parameter('wheel_diameter', 0.066)     # meters
        self.declare_parameter('wheelbase', 0.257)          # meters (TT-02)
        self.declare_parameter('publish_rate', 30.0)        # Hz
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.enc_a_pin = self.get_parameter('encoder_a_pin').value
        self.enc_b_pin = self.get_parameter('encoder_b_pin').value
        self.ppr = self.get_parameter('pulses_per_rev').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Derived constants
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.distance_per_pulse = self.wheel_circumference / self.ppr

        # --- State variables ---
        self.pulse_count = 0
        self.last_pulse_count = 0
        self.direction = 1          # 1=forward, -1=backward
        self.last_time = self.get_clock().now()

        # Odometry state (2D: x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0

        # --- pigpio setup ---
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Failed to connect to pigpiod!')
            raise RuntimeError('pigpiod not running')

        # Set encoder pins as input with pull-up
        self.pi.set_mode(self.enc_a_pin, pigpio.INPUT)
        self.pi.set_mode(self.enc_b_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.enc_a_pin, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.enc_b_pin, pigpio.PUD_UP)

        # Register callback on A channel rising edge
        self._cb_a = self.pi.callback(
            self.enc_a_pin, pigpio.RISING_EDGE, self._encoder_callback
        )

        # --- ROS publishers ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Timer for periodic publishing ---
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._publish_odom)

        self.get_logger().info(
            f'Encoder Odometry ready: GPIO A={self.enc_a_pin}, B={self.enc_b_pin}, '
            f'PPR={self.ppr}, wheel_dia={self.wheel_diameter}m, '
            f'rate={self.publish_rate}Hz'
        )

    def _encoder_callback(self, gpio, level, tick):
        """Called on every rising edge of encoder A channel."""
        b_state = self.pi.read(self.enc_b_pin)
        if b_state == 0:
            # A rises while B is low -> forward
            self.direction = 1
        else:
            # A rises while B is high -> backward
            self.direction = -1
        self.pulse_count += 1

    def _publish_odom(self):
        """Timer callback: compute odometry and publish."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        # Calculate distance traveled since last publish
        pulses = self.pulse_count - self.last_pulse_count
        self.last_pulse_count = self.pulse_count
        distance = pulses * self.distance_per_pulse * self.direction if pulses > 0 else 0.0

        # Calculate velocity
        self.vx = distance / dt

        # Update 2D pose (straight-line approximation for single encoder)
        # Without steering angle feedback, we assume straight-line motion
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

        # Build Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self._yaw_to_quaternion(self.theta)

        # Velocity
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Covariance (diagonal, moderate uncertainty)
        # pose covariance [x, y, z, roll, pitch, yaw]
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[14] = 1e6   # z (no info)
        odom_msg.pose.covariance[21] = 1e6   # roll
        odom_msg.pose.covariance[28] = 1e6   # pitch
        odom_msg.pose.covariance[35] = 0.05  # yaw

        # twist covariance
        odom_msg.twist.covariance[0] = 0.01   # vx
        odom_msg.twist.covariance[7] = 1e6    # vy
        odom_msg.twist.covariance[35] = 1e6   # vyaw

        self.odom_pub.publish(odom_msg)

        # Broadcast TF: odom -> base_link
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = self._yaw_to_quaternion(self.theta)
            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        """Convert yaw angle to geometry_msgs Quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def destroy_node(self):
        self.get_logger().info('Shutting down encoder odometry...')
        self._cb_a.cancel()
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
