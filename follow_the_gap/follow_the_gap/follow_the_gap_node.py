#!/usr/bin/env python3
"""
Follow the Gap Node
  - ギャップ検出 (FTG)
  - Pure Pursuit ステアリング制御
  - 速度 PID（Ziegler-Nichols オーバーシュートなし）
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_srvs.srv import SetBool


class PIDController:
    """ZN法（オーバーシュートなし）速度PIDコントローラ"""

    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt
        self._prev_error = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return float(np.clip(output, self.out_min, self.out_max))


class FollowTheGapNode(Node):

    def __init__(self):
        super().__init__('follow_the_gap_node')

        # ── パラメータ宣言 ──────────────────────────────
        self.declare_parameter('wheelbase',          0.257)
        self.declare_parameter('bubble_radius',      0.3)
        self.declare_parameter('scan_range_min',     0.1)
        self.declare_parameter('scan_range_max',     3.5)
        self.declare_parameter('scan_angle_min',    -2.35)
        self.declare_parameter('scan_angle_max',     2.35)
        self.declare_parameter('k_lookahead',        0.5)
        self.declare_parameter('lookahead_min',      0.3)
        self.declare_parameter('lookahead_max',      2.0)
        self.declare_parameter('max_steering_angle', 0.4)
        self.declare_parameter('speed_target',       1.0)
        self.declare_parameter('speed_min',          0.3)
        self.declare_parameter('zn_ku',              2.0)
        self.declare_parameter('zn_tu',              0.5)
        self.declare_parameter('pid_output_max',     3.0)
        self.declare_parameter('pid_output_min',     0.0)

        self._load_params()

        # ── 状態変数 ──────────────────────────────────
        self._enabled    = False
        self._v_current  = 0.0
        self._prev_stamp = None

        # ── PID 初期化 ────────────────────────────────
        self._init_pid()

        # ── Pub / Sub / Service ───────────────────────
        self._drive_pub = self.create_publisher(
            AckermannDriveStamped, 'drive', 10)

        self._scan_sub = self.create_subscription(
            LaserScan, 'scan', self._scan_callback, 10)

        self._odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_callback, 10)

        self._enable_srv = self.create_service(
            SetBool, '~/enable', self._enable_callback)

        self.get_logger().info(
            f'follow_the_gap_node 起動完了 '
            f'[Kp={self._pid.kp:.4f} Ki={self._pid.ki:.4f} Kd={self._pid.kd:.4f}]'
        )
        self.get_logger().info('enable サービスで走行開始: ros2 service call ~/enable std_srvs/srv/SetBool "{data: true}"')

    # ──────────────────────────────────────────────────
    def _load_params(self):
        self._wheelbase          = self.get_parameter('wheelbase').value
        self._bubble_radius      = self.get_parameter('bubble_radius').value
        self._scan_range_min     = self.get_parameter('scan_range_min').value
        self._scan_range_max     = self.get_parameter('scan_range_max').value
        self._scan_angle_min     = self.get_parameter('scan_angle_min').value
        self._scan_angle_max     = self.get_parameter('scan_angle_max').value
        self._k_lookahead        = self.get_parameter('k_lookahead').value
        self._lookahead_min      = self.get_parameter('lookahead_min').value
        self._lookahead_max      = self.get_parameter('lookahead_max').value
        self._max_steer          = self.get_parameter('max_steering_angle').value
        self._speed_target       = self.get_parameter('speed_target').value
        self._speed_min          = self.get_parameter('speed_min').value
        self._zn_ku              = self.get_parameter('zn_ku').value
        self._zn_tu              = self.get_parameter('zn_tu').value
        self._pid_out_max        = self.get_parameter('pid_output_max').value
        self._pid_out_min        = self.get_parameter('pid_output_min').value

    def _init_pid(self):
        """ZN法（オーバーシュートなし）でPIDゲインを自動計算"""
        ku = self._zn_ku
        tu = self._zn_tu
        kp = 0.20 * ku
        ki = 0.40 * ku / tu
        kd = 0.066 * ku * tu
        self._pid = PIDController(kp, ki, kd, self._pid_out_min, self._pid_out_max)
        self.get_logger().info(
            f'ZN法ゲイン計算: Ku={ku} Tu={tu} → Kp={kp:.4f} Ki={ki:.4f} Kd={kd:.4f}')

    # ──────────────────────────────────────────────────
    def _enable_callback(self, request, response):
        self._enabled = request.data
        if self._enabled:
            self._pid.reset()
            self.get_logger().info('★ FTG 走行 開始')
        else:
            self._publish_stop()
            self.get_logger().info('■ FTG 走行 停止')
        response.success = True
        response.message = 'enabled' if self._enabled else 'disabled'
        return response

    def _odom_callback(self, msg: Odometry):
        self._v_current = msg.twist.twist.linear.x

    # ──────────────────────────────────────────────────
    def _scan_callback(self, msg: LaserScan):
        if not self._enabled:
            return

        # dt 計算
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = (now - self._prev_stamp) if self._prev_stamp is not None else 0.033
        self._prev_stamp = now
        dt = max(dt, 1e-4)

        # ── Step 1: 前処理 ────────────────────────────
        ranges = np.array(msg.ranges, dtype=np.float32)
        angle_min  = msg.angle_min
        angle_inc  = msg.angle_increment
        n          = len(ranges)
        angles     = angle_min + np.arange(n) * angle_inc

        # 有効角度範囲マスク
        angle_mask = (angles >= self._scan_angle_min) & (angles <= self._scan_angle_max)

        # inf / nan / 範囲外 を scan_range_max でクリップ
        ranges = np.where(np.isfinite(ranges), ranges, self._scan_range_max)
        ranges = np.clip(ranges, self._scan_range_min, self._scan_range_max)

        # 有効範囲外を 0 にマスク
        proc = np.where(angle_mask, ranges, 0.0)

        # ── Step 2: 最近傍点検出 ──────────────────────
        valid_mask = proc > 0
        if not np.any(valid_mask):
            self.get_logger().warn('有効スキャン点なし')
            self._publish_stop()
            return

        masked_ranges = np.where(valid_mask, proc, np.inf)
        closest_idx   = int(np.argmin(masked_ranges))

        # ── Step 3: 安全バブル処理 ────────────────────
        bubble_angle = math.atan2(self._bubble_radius, max(proc[closest_idx], 0.01))
        bubble_steps = int(bubble_angle / angle_inc)
        lo = max(0,     closest_idx - bubble_steps)
        hi = min(n - 1, closest_idx + bubble_steps)
        proc[lo:hi+1] = 0.0

        # ── Step 4: 最大ギャップ検出 ──────────────────
        gap_start, gap_end = self._find_max_gap(proc)
        if gap_start is None:
            self.get_logger().warn('ギャップ検出失敗')
            self._publish_stop()
            return

        # ── Step 5: 目標点選定（最遠点）─────────────────
        gap_ranges = proc[gap_start:gap_end+1]
        best_local = int(np.argmax(gap_ranges))
        target_idx = gap_start + best_local
        target_angle  = angles[target_idx]
        target_dist   = proc[target_idx]

        # ── Step 6: Pure Pursuit ステアリング計算 ───────
        v = max(abs(self._v_current), 0.01)
        lookahead = float(np.clip(self._k_lookahead * v,
                                   self._lookahead_min,
                                   self._lookahead_max))

        alpha   = target_angle          # laser フレームでの目標点角度
        steer   = math.atan2(2.0 * self._wheelbase * math.sin(alpha), lookahead)
        steer   = float(np.clip(steer, -self._max_steer, self._max_steer))

        # ── Step 7: 速度 PID ──────────────────────────
        error   = self._speed_target - self._v_current
        speed   = self._pid.compute(error, dt)
        speed   = max(speed, self._speed_min)

        # ── Publish ───────────────────────────────────
        self._publish_drive(steer, speed)

        self.get_logger().debug(
            f'target_angle={math.degrees(target_angle):.1f}° '
            f'dist={target_dist:.2f}m '
            f'lookahead={lookahead:.2f}m '
            f'steer={math.degrees(steer):.1f}° '
            f'v_cur={self._v_current:.2f} speed_cmd={speed:.2f}'
        )

    # ──────────────────────────────────────────────────
    def _find_max_gap(self, proc):
        """連続する非ゼロ要素の最長区間を返す"""
        best_start = best_end = None
        best_len   = 0
        cur_start  = None

        for i, val in enumerate(proc):
            if val > 0:
                if cur_start is None:
                    cur_start = i
            else:
                if cur_start is not None:
                    length = i - cur_start
                    if length > best_len:
                        best_len  = length
                        best_start = cur_start
                        best_end   = i - 1
                    cur_start = None

        # 末尾まで続いていた場合
        if cur_start is not None:
            length = len(proc) - cur_start
            if length > best_len:
                best_start = cur_start
                best_end   = len(proc) - 1

        return best_start, best_end

    # ──────────────────────────────────────────────────
    def _publish_drive(self, steering_angle, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = steering_angle
        msg.drive.speed          = speed
        self._drive_pub.publish(msg)

    def _publish_stop(self):
        self._publish_drive(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
