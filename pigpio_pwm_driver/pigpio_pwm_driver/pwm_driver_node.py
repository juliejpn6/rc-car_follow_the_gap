import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import pigpio
import math


class PwmDriverNode(Node):
    def __init__(self):
        super().__init__('pwm_driver_node')

        # Declare parameters matching the jog program settings
        self.declare_parameter('steer_pin', 13)        # Hardware PWM pin
        self.declare_parameter('throttle_pin', 12)     # Hardware PWM pin
        self.declare_parameter('pwm_frequency', 70)    # Hz
        self.declare_parameter('neutral_duty_steer', 10.00)   # % duty at 70Hz
        self.declare_parameter('neutral_duty_throttle', 10.48) # % duty at 70Hz
        self.declare_parameter('max_steering_angle', 0.36)  # radians
        self.declare_parameter('max_speed', 3.0)            # m/s
        self.declare_parameter('steer_duty_range', 1.0)     # +/- duty % from neutral
        self.declare_parameter('throttle_duty_range', 2.0)  # +/- duty % from neutral
        self.declare_parameter('safety_timeout', 0.5)       # seconds

        # Get parameters
        self.steer_pin = self.get_parameter('steer_pin').value
        self.throttle_pin = self.get_parameter('throttle_pin').value
        self.pwm_freq = self.get_parameter('pwm_frequency').value
        self.neutral_steer = self.get_parameter('neutral_duty_steer').value
        self.neutral_throttle = self.get_parameter('neutral_duty_throttle').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_speed = self.get_parameter('max_speed').value
        self.steer_range = self.get_parameter('steer_duty_range').value
        self.throttle_range = self.get_parameter('throttle_duty_range').value
        self.safety_timeout = self.get_parameter('safety_timeout').value

        # Safety limits for duty cycle (%)
        self.PWM_SAFE_MIN = 7.50
        self.PWM_SAFE_MAX = 13.00

        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Failed to connect to pigpiod!')
            raise RuntimeError('pigpiod not connected')
        self.get_logger().info('Connected to pigpiod')

        # Set GPIO modes and neutral position
        self.pi.set_mode(self.steer_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.throttle_pin, pigpio.OUTPUT)
        self.set_neutral()

        # Subscribe to /drive topic
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )

        # Safety timer
        self.last_cmd_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info(
            f'PWM Driver ready: steer=GPIO{self.steer_pin}, '
            f'throttle=GPIO{self.throttle_pin}, freq={self.pwm_freq}Hz, '
            f'neutral_steer={self.neutral_steer}%, neutral_throttle={self.neutral_throttle}%'
        )

    def duty100(self, rate):
        """Convert duty % to pigpio hardware_PWM value (0-1000000)."""
        rate = max(self.PWM_SAFE_MIN, min(self.PWM_SAFE_MAX, rate))
        return int(rate * 10000)

    def drive_callback(self, msg: AckermannDriveStamped):
        self.last_cmd_time = self.get_clock().now()

        steering_angle = msg.drive.steering_angle  # radians
        speed = msg.drive.speed  # m/s

        # Convert steering angle to duty cycle
        steering_angle = max(-self.max_steering_angle,
                           min(self.max_steering_angle, steering_angle))
        steer_ratio = steering_angle / self.max_steering_angle
        steer_duty = self.neutral_steer + (steer_ratio * self.steer_range)

        # Convert speed to duty cycle
        speed = max(-self.max_speed, min(self.max_speed, speed))
        speed_ratio = speed / self.max_speed
        throttle_duty = self.neutral_throttle - (speed_ratio * self.throttle_range)

        # Apply hardware PWM
        self.pi.hardware_PWM(self.steer_pin, self.pwm_freq, self.duty100(steer_duty))
        self.pi.hardware_PWM(self.throttle_pin, self.pwm_freq, self.duty100(throttle_duty))

        self.get_logger().debug(
            f'steer={steering_angle:.2f}rad -> {steer_duty:.2f}%, '
            f'speed={speed:.2f}m/s -> {throttle_duty:.2f}%'
        )

    def safety_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.safety_timeout:
            self.set_neutral()

    def set_neutral(self):
        self.pi.hardware_PWM(self.steer_pin, self.pwm_freq, self.duty100(self.neutral_steer))
        self.pi.hardware_PWM(self.throttle_pin, self.pwm_freq, self.duty100(self.neutral_throttle))

    def destroy_node(self):
        self.get_logger().info('Shutting down, setting neutral...')
        self.set_neutral()
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PwmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
