#!/usr/bin/env python3
"""
ackermann_odom_publisher.py

/odom_raw (encoder直線距離) + /ackermann_cmd (ステアリング角) を入力として
Ackermann運動学モデルで自己位置 (x, y, theta) を計算し /odom を配信する。

配信頻度: /odom_raw の受信頻度に同期（タイマー駆動ではなくコールバック駆動）

TF配信: publish_tf パラメータで ON/OFF 可能（デフォルト: True）
        encoder_odom_node 側の TF を無効化した上でこちらを True にすること。
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class AckermannOdomPublisher(Node):

    def __init__(self):
        super().__init__('ackermann_odom_publisher')

        # ── パラメータ宣言 ──────────────────────────────
        self.declare_parameter('wheelbase',         0.257)   # ホイールベース [m]
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('base_frame',        'base_link')
        self.declare_parameter('odom_raw_topic',    '/odom_raw')
        self.declare_parameter('cmd_topic',         '/ackermann_cmd')
        self.declare_parameter('odom_out_topic',    '/odom')
        self.declare_parameter('publish_tf',        True)    # TF配信 ON/OFF

        self._L          = self.get_parameter('wheelbase').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._pub_tf     = self.get_parameter('publish_tf').value

        # ── 状態変数 ──────────────────────────────────
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0

        self._prev_x  = None   # odom_raw の直前 x 値（差分計算用）
        self._delta   = 0.0    # 最新ステアリング角 [rad]

        # ── Publisher / Subscriber / TF ───────────────
        odom_out = self.get_parameter('odom_out_topic').value
        self._odom_pub = self.create_publisher(Odometry, odom_out, 10)

        if self._pub_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        odom_raw = self.get_parameter('odom_raw_topic').value
        cmd_top  = self.get_parameter('cmd_topic').value

        self._odom_raw_sub = self.create_subscription(
            Odometry, odom_raw, self._odom_raw_callback, 10)

        self._cmd_sub = self.create_subscription(
            AckermannDriveStamped, cmd_top, self._cmd_callback, 10)

        self.get_logger().info(
            f'ackermann_odom_publisher 起動完了 '
            f'[wheelbase={self._L}m publish_tf={self._pub_tf}]'
        )
        self.get_logger().info(
            f'  入力: {odom_raw} + {cmd_top}'
        )
        self.get_logger().info(
            f'  出力: {odom_out} (TF: {"odom->base_link" if self._pub_tf else "無効"})'
        )

    # ──────────────────────────────────────────────────
    def _cmd_callback(self, msg: AckermannDriveStamped):
        """ステアリング角を更新（最新値を保持するだけ）"""
        self._delta = msg.drive.steering_angle

    # ──────────────────────────────────────────────────
    def _odom_raw_callback(self, msg: Odometry):
        """
        /odom_raw 受信をトリガーに Ackermann 運動学で位置更新し /odom を配信。
        配信頻度は /odom_raw の受信頻度に完全同期する。
        """
        now = msg.header.stamp  # encoder タイムスタンプをそのまま使用

        # ── 直線移動量 ds を計算 ──────────────────────
        # encoder_odom_node は累積距離を x に格納している前提
        raw_x = msg.pose.pose.position.x

        if self._prev_x is None:
            # 初回受信: 差分計算不可のため位置だけ記録
            self._prev_x = raw_x
            return

        ds = raw_x - self._prev_x   # 今回の移動距離 [m]
        self._prev_x = raw_x

        # ── Ackermann 運動学モデル ────────────────────
        # dx     = ds * cos(theta)
        # dy     = ds * sin(theta)
        # dtheta = ds / L * tan(delta)
        delta = self._delta
        dtheta = 0.0
        if abs(self._L) > 1e-6:
            dtheta = ds / self._L * math.tan(delta)

        self._x     += ds * math.cos(self._theta)
        self._y     += ds * math.sin(self._theta)
        self._theta += dtheta

        # theta を -pi 〜 +pi に正規化
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        # ── /odom 配信 ────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._yaw_to_quaternion(self._theta)

        # 速度は odom_raw の値をそのまま流用
        odom.twist.twist.linear.x  = msg.twist.twist.linear.x
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = (
            msg.twist.twist.linear.x / self._L * math.tan(delta)
            if abs(self._L) > 1e-6 else 0.0
        )

        # 共分散（Ackermann補正済みのため odom_raw より精度向上）
        odom.pose.covariance[0]  = 0.005   # x
        odom.pose.covariance[7]  = 0.005   # y
        odom.pose.covariance[14] = 1e6     # z
        odom.pose.covariance[21] = 1e6     # roll
        odom.pose.covariance[28] = 1e6     # pitch
        odom.pose.covariance[35] = 0.02    # yaw
        odom.twist.covariance[0]  = 0.005
        odom.twist.covariance[7]  = 1e6
        odom.twist.covariance[35] = 0.02

        self._odom_pub.publish(odom)

        # ── TF 配信 ───────────────────────────────────
        if self._pub_tf:
            t = TransformStamped()
            t.header.stamp    = now
            t.header.frame_id = self._odom_frame
            t.child_frame_id  = self._base_frame

            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation = self._yaw_to_quaternion(self._theta)

            self._tf_broadcaster.sendTransform(t)

    # ──────────────────────────────────────────────────
    @staticmethod
    def _yaw_to_quaternion(yaw: float):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = AckermannOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
