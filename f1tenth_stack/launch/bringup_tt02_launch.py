# =============================================================================
# TT-02 Integrated Bringup Launch File
# =============================================================================
# All nodes for autonomous driving on Tamiya TT-02 chassis:
#
# [Sensing]
#   - sllidar_ros2        : RPLidar A1-M8 LiDAR -> /scan
#   - encoder_odometry    : Optical encoder -> /odom + TF(odom -> base_link)
#
# [Control]
#   - pigpio_pwm_driver   : /ackermann_cmd -> Steering & throttle PWM
#   - ackermann_mux       : /drive topic multiplexer (nav / teleop priority)
#
# [Planning]
#   - follow_the_gap      : /scan -> FTG + Pure Pursuit + Speed PID -> /drive
#                           (starts DISABLED, enable via service call)
#
# [TF]
#   - static TF           : base_link -> laser
#
# Usage:
#   ros2 launch f1tenth_stack bringup_tt02_launch.py
#
# Start driving:
#   ros2 service call /follow_the_gap_node/enable std_srvs/srv/SetBool "{data: true}"
#
# Stop driving:
#   ros2 service call /follow_the_gap_node/enable std_srvs/srv/SetBool "{data: false}"
# =============================================================================

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # =========================================================================
    # Config file paths
    # =========================================================================
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )
    ftg_config = os.path.join(
        get_package_share_directory('follow_the_gap'),
        'config',
        'follow_the_gap.yaml'
    )

    # =========================================================================
    # Launch arguments
    # =========================================================================
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Path to ackermann mux config'
    )
    ftg_la = DeclareLaunchArgument(
        'ftg_config',
        default_value=ftg_config,
        description='Path to follow_the_gap config'
    )
    lidar_serial_port_la = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/ttyUSB0',
        description='LiDAR serial port'
    )
    lidar_frame_id_la = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='laser',
        description='LiDAR TF frame ID'
    )

    # =========================================================================
    # Sensing Nodes
    # =========================================================================

    # --- LiDAR (RPLidar A1-M8) ---
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': 115200,
            'frame_id': LaunchConfiguration('lidar_frame_id'),
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }]
    )

    # --- Encoder Odometry (直線距離のみ → /odom_raw に配信) ---
    encoder_odom_node = Node(
        package='encoder_odometry',
        executable='encoder_odom_node',
        name='encoder_odom_node',
        output='screen',
        parameters=[{
            'encoder_a_pin': 22,
            'encoder_b_pin': 27,
            'pulses_per_rev': 36,
            'wheel_diameter': 0.066,
            'wheelbase': 0.257,
            'publish_rate': 30.0,
            'publish_tf': False,   # TF配信は ackermann_odom_publisher に委譲
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }],
        remappings=[('/odom', '/odom_raw')],  # 直線odomは /odom_raw へ
    )

    # --- Ackermann Odometry (Ackermann補正済み /odom + TF 配信) ---
    ackermann_odom_node = Node(
        package='encoder_odometry',
        executable='ackermann_odom_publisher',
        name='ackermann_odom_publisher',
        output='screen',
        parameters=[{
            'wheelbase':      0.257,
            'odom_frame':     'odom',
            'base_frame':     'base_link',
            'odom_raw_topic': '/odom_raw',
            'cmd_topic':      '/ackermann_cmd',
            'odom_out_topic': '/odom',
            'publish_tf':     True,  # TF: odom -> base_link を担当
        }],
    )

    # =========================================================================
    # Control Nodes
    # =========================================================================

    # --- PWM Driver (steering & throttle) ---
    pwm_driver_node = Node(
        package='pigpio_pwm_driver',
        executable='pwm_driver_node',
        name='pwm_driver_node',
        remappings=[('/drive', '/ackermann_cmd')],
        output='screen',
        parameters=[{
            'steer_pin': 13,
            'throttle_pin': 12,
            'pwm_frequency': 70,
            'neutral_duty_steer': 10.00,
            'neutral_duty_throttle': 10.48,
            'max_steering_angle': 0.36,
            'max_speed': 3.0,
            'steer_duty_range': 2.0,
            'throttle_duty_range': 2.0,
            'safety_timeout': 0.5,
        }]
    )

    # --- Ackermann Mux ---
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        output='screen',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'drive')]
    )

    # =========================================================================
    # Planning Nodes
    # =========================================================================

    # --- Follow the Gap (starts DISABLED for safety) ---
    follow_the_gap_node = Node(
        package='follow_the_gap',
        executable='follow_the_gap_node',
        name='follow_the_gap_node',
        output='screen',
        parameters=[LaunchConfiguration('ftg_config')]
    )

    # =========================================================================
    # Static TF: base_link -> laser
    # =========================================================================
    # Measured values for TT-02:
    #   x = 0.23m (23cm forward from rear axle)
    #   y = 0.0m  (centered on vehicle)
    #   z = 0.12m (12cm above ground)
    #   yaw = 3.14159 rad (LiDAR mounted 180deg inverted)
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '--x', '0.23',
            '--y', '0.0',
            '--z', '0.12',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '3.14159',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ]
    )

    # =========================================================================
    # Build LaunchDescription
    # =========================================================================
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(mux_la)
    ld.add_action(ftg_la)
    ld.add_action(lidar_serial_port_la)
    ld.add_action(lidar_frame_id_la)

    # Sensing
    ld.add_action(lidar_node)
    ld.add_action(encoder_odom_node)
    ld.add_action(ackermann_odom_node)

    # Control
    ld.add_action(pwm_driver_node)
    ld.add_action(ackermann_mux_node)

    # Planning
    ld.add_action(follow_the_gap_node)

    # Static TF
    ld.add_action(static_tf_base_to_laser)

    return ld
