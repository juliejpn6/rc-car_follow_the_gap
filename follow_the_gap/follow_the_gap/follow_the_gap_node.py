#!/usr/bin/env python3
"""
Follow the Gap Node v4.2
  np.roll(n//2) で物理前方をindex中央に配置
  angles[center]=0=前方、正=左、負=右
  3点メディアンフィルタによるスパイクノイズ除去
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class PIDController:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp=kp; self.ki=ki; self.kd=kd
        self.out_min=out_min; self.out_max=out_max
        self._integral=0.0; self._prev_error=0.0

    def reset(self):
        self._integral=0.0; self._prev_error=0.0

    def compute(self, error, dt):
        if dt<=0.0: return 0.0
        self._integral += error*dt
        d = (error-self._prev_error)/dt
        self._prev_error = error
        return float(np.clip(
            self.kp*error + self.ki*self._integral + self.kd*d,
            self.out_min, self.out_max))


class FollowTheGapNode(Node):

    def __init__(self):
        super().__init__('follow_the_gap_node')
        self.declare_parameter('wheelbase',             0.257)
        self.declare_parameter('bubble_radius',         0.12)
        self.declare_parameter('scan_range_min',        0.10)
        self.declare_parameter('scan_range_max',        3.5)
        self.declare_parameter('front_angle_width',     1.57)
        self.declare_parameter('emergency_angle_width', 0.26)
        self.declare_parameter('k_lookahead',           0.5)
        self.declare_parameter('lookahead_min',         0.5)
        self.declare_parameter('lookahead_max',         0.8)
        self.declare_parameter('max_steering_angle',    0.4)
        self.declare_parameter('speed_target',          0.5)
        self.declare_parameter('speed_min',             0.3)
        self.declare_parameter('use_speed_pid',         False)
        self.declare_parameter('front_stop_distance',   0.35)
        self.declare_parameter('max_target_angle',      0.61)
        self.declare_parameter('steer_sign',            -1)
        self.declare_parameter('zn_ku',                 2.0)
        self.declare_parameter('zn_tu',                 0.5)
        self.declare_parameter('pid_output_max',        1.0)
        self.declare_parameter('pid_output_min',        0.0)


        self._load_params()
        self._init_pid()
        self._enabled=False; self._v_current=0.0; self._prev_stamp=None
        self._steer_ema=0.0
        self._gap_center_prev=-1
        self._alpha_prev=0.0
        self._steer_prev=0.0
        self._alpha_filtered=0.0
        self._wall_diff_hist=[0.0, 0.0, 0.0]
        self._prev_was_corner=False
        # ── ログ制御 ──────────────────────────────────
        self._log_last_summary     = 0.0
        self._LOG_SUMMARY_INTERVAL = 1.0
        self._log_prev             = {}
        self._LOG_STEER_TH = 3.0
        self._LOG_FRONT_TH = 0.15
        self._LOG_GAP_W_TH = 15.0

        self._drive_pub  = self.create_publisher(AckermannDriveStamped,'drive',10)
        self._marker_pub = self.create_publisher(MarkerArray, '/ftg_markers', 10)
        self._scan_sub   = self.create_subscription(LaserScan,'scan',self._scan_callback,10)
        self._odom_sub   = self.create_subscription(Odometry,'odom',self._odom_callback,10)
        self._enable_srv = self.create_service(
            SetBool, '/follow_the_gap_node/enable', self._enable_callback)
        self.get_logger().info(
            f'follow_the_gap_node v4.1 起動完了 '
            f'speed={self._speed_target} '
            f'steer_sign={self._steer_sign} '
            f'front=±{math.degrees(self._front_angle_width):.0f}deg')
        self.get_logger().info(
            'enable: ros2 service call /follow_the_gap_node/enable '
            'std_srvs/srv/SetBool "{data: true}"')

    def _load_params(self):
        self._wheelbase        = self.get_parameter('wheelbase').value
        self._bubble_radius    = self.get_parameter('bubble_radius').value
        self._scan_range_min   = self.get_parameter('scan_range_min').value
        self._scan_range_max   = self.get_parameter('scan_range_max').value
        self._front_angle_width= self.get_parameter('front_angle_width').value
        self._emg_angle_width  = self.get_parameter('emergency_angle_width').value
        self._k_lookahead      = self.get_parameter('k_lookahead').value
        self._lookahead_min    = self.get_parameter('lookahead_min').value
        self._lookahead_max    = self.get_parameter('lookahead_max').value
        self._max_steer        = self.get_parameter('max_steering_angle').value
        self._speed_target     = self.get_parameter('speed_target').value
        self._speed_min        = self.get_parameter('speed_min').value
        self._use_speed_pid    = self.get_parameter('use_speed_pid').value
        self._front_stop_dist  = self.get_parameter('front_stop_distance').value
        self._max_target_angle = self.get_parameter('max_target_angle').value
        self._steer_sign       = self.get_parameter('steer_sign').value
        self._zn_ku            = self.get_parameter('zn_ku').value
        self._zn_tu            = self.get_parameter('zn_tu').value
        self._pid_out_max      = self.get_parameter('pid_output_max').value
        self._pid_out_min      = self.get_parameter('pid_output_min').value


    def _init_pid(self):
        ku=self._zn_ku; tu=self._zn_tu
        kp=0.20*ku; ki=0.40*ku/tu; kd=0.066*ku*tu
        self._pid=PIDController(kp,ki,kd,self._pid_out_min,self._pid_out_max)
        self.get_logger().info(
            f'ZN法ゲイン: Ku={ku} Tu={tu} → '
            f'Kp={kp:.4f} Ki={ki:.4f} Kd={kd:.4f}')

    def _enable_callback(self, req, res):
        self._enabled=req.data
        if self._enabled:
            self._pid.reset()
            self._steer_ema=0.0
            self._gap_center_prev=-1
            self._alpha_filtered=0.0
            self._steer_prev=0.0
            self._alpha_prev=0.0
            self._wall_diff_hist=[0.0, 0.0, 0.0]
            self._prev_was_corner=False
            self._log_prev         = {}
            self._log_last_summary = 0.0
            self.get_logger().info('★ FTG 走行 開始')
        else:
            self._publish_stop()
            self.get_logger().info('■ FTG 走行 停止')
        res.success=True; res.message='enabled' if self._enabled else 'disabled'
        return res

    def _odom_callback(self, msg):
        self._v_current = msg.twist.twist.linear.x

    def _scan_callback(self, msg: LaserScan):
        if not self._enabled:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        dt  = (now-self._prev_stamp) if self._prev_stamp is not None else 0.033
        self._prev_stamp = now
        dt  = max(dt, 1e-4)

        # ── Step 1: 前処理 + 配列ロール ─────────────────────────
        ranges_raw = np.array(msg.ranges, dtype=np.float32)
        angle_inc  = msg.angle_increment
        n          = len(ranges_raw)

        ranges_raw = np.where(np.isfinite(ranges_raw), ranges_raw, self._scan_range_max)
        ranges_raw = np.clip(ranges_raw, 0.0, self._scan_range_max)  # 下限0でロール前はクリップしない

        # ★ ロール: n//2 シフトで物理前方を配列中央へ
        shift  = n // 2
        ranges = np.roll(ranges_raw, shift)
        center = n // 2
        angles = (np.arange(n) - center) * angle_inc

        # ── メディアンフィルタ（3点、numpyのみ）─────────────
        # 隣接3点の中央値でスパイクノイズを除去
        r_left  = np.roll(ranges,  1)
        r_right = np.roll(ranges, -1)
        ranges  = np.median(np.stack([r_left, ranges, r_right], axis=0), axis=0)

        # ── 有効範囲マスク（前方 ±front_angle_width）────────────
        front_mask = np.abs(angles) <= self._front_angle_width
        proc = np.where(front_mask, ranges, 0.0)

        # ── scan_range_min 以下を無効化（マスク後に適用）────────
        proc = np.where(proc >= self._scan_range_min, proc, 0.0)

        # ── 緊急停止判定（前方 ±emg_angle_width）────────────────
        emg_mask   = np.abs(angles) <= self._emg_angle_width
        emg_ranges = np.where(emg_mask & (ranges > 0), ranges, np.inf)
        front_min  = float(np.min(emg_ranges)) if np.any(emg_mask) else self._scan_range_max
        if front_min <= self._front_stop_dist:
            self.get_logger().warn(
                f'★ 緊急停止: 前方{front_min:.2f}m < 停止距離{self._front_stop_dist:.2f}m')
            self._publish_stop()
            return

        # ── Step 2: 最近傍点検出 ──────────────────────────────────
        valid_mask = proc > 0
        if not np.any(valid_mask):
            self.get_logger().warn('有効スキャン点なし')
            self._publish_stop()
            return
        masked       = np.where(valid_mask, proc, np.inf)
        closest_idx  = int(np.argmin(masked))
        closest_dist = float(ranges[closest_idx])

        # ── Step 3: 安全バブル ────────────────────────────────────
        bubble_angle = math.atan2(self._bubble_radius, max(proc[closest_idx], 0.01))
        bubble_steps = int(bubble_angle / angle_inc)
        lo = max(0,     closest_idx-bubble_steps)
        hi = min(n-1,   closest_idx+bubble_steps)
        proc[lo:hi+1]  = 0.0

        # ── Step 4: 最大ギャップ ──────────────────────────────────
        gap_start, gap_end = self._find_max_gap(proc)
        if gap_start is None:
            self.get_logger().warn('ギャップ検出失敗')
            self._publish_stop()
            return

        # ── Step 5: 目標点選定（直線/コーナー切替）──────────────
        prev_corner = self._prev_was_corner
        if prev_corner:
            is_straight = front_min > 2.0   # CRN→STR: より広い空間が必要
        else:
            is_straight = front_min > 1.3   # STR→CRN: より早めにコーナー検出
        self._prev_was_corner = not is_straight

        if is_straight:
            # 直線モード: 左右壁距離差でステアリング
            if prev_corner:
                self._wall_diff_hist = [0.0, 0.0, 0.0]
            left_idx  = int(np.clip(center + int(math.radians(45)/angle_inc), 0, n-1))
            right_idx = int(np.clip(center - int(math.radians(45)/angle_inc), 0, n-1))
            d_left  = float(ranges[left_idx])  if ranges[left_idx]  > 0.1 else self._scan_range_max
            d_right = float(ranges[right_idx]) if ranges[right_idx] > 0.1 else self._scan_range_max
            wall_diff = d_left - d_right
            self._wall_diff_hist.pop(0)
            self._wall_diff_hist.append(wall_diff)
            wall_diff_avg = sum(self._wall_diff_hist) / len(self._wall_diff_hist)
            alpha     = float(np.clip(math.radians(wall_diff_avg * 10.0),
                                      -math.radians(10), math.radians(10)))
            target_dist = front_min
            self._gap_center_prev = gap_start + (gap_end - gap_start) // 2
            target_idx  = int(np.clip(int(self._gap_center_prev), 0, n-1))
            mode = 'STR'
        else:
            # コーナーモード: ギャップ中央 + レートリミッタ
            gap_center_now = gap_start + (gap_end - gap_start) // 2
            if front_min < 0.6:
                max_rate = 180
            elif front_min < 0.8:
                max_rate = 180
            elif front_min < 1.2:
                max_rate = 120
            else:
                max_rate = 90
            self._gap_center_prev = gap_center_now
            target_idx  = int(np.clip(int(self._gap_center_prev), 0, n-1))
            alpha       = angles[target_idx]
            target_dist = proc[target_idx] if proc[target_idx] > 0 else self._scan_range_max
            mode = 'CRN'

        max_alpha = self._max_target_angle
        alpha     = float(np.clip(alpha, -max_alpha, max_alpha))

        # 急激な符号逆転を検出して無視（直線モード時のみ）
        if is_straight:
            alpha_change    = abs(alpha - self._alpha_prev)
            alpha_sign_flip = (alpha * self._alpha_prev < 0)
            if alpha_sign_flip and alpha_change > math.radians(15.0):
                alpha = self._alpha_prev
        self._alpha_prev = alpha

        # デッドバンド（直線モード時のみ ±5°以内は直進）
        if is_straight and abs(alpha) < math.radians(5.0):
            alpha = 0.0
        target_dist = proc[target_idx] if proc[target_idx] > 0 else self._scan_range_max

        # ── Step 6: Pure Pursuit ─────────────────────────────────
        v = max(abs(self._v_current), 0.01)
        lookahead = float(np.clip(
            self._k_lookahead*v, self._lookahead_min, self._lookahead_max))
        steer = math.atan2(2.0*self._wheelbase*math.sin(alpha), lookahead)
        steer = float(np.clip(
            self._steer_sign * steer, -self._max_steer, self._max_steer))

        # ── ステアリングレートリミッタ ───────────────────────────
        steer_abs = abs(steer)
        if steer_abs > math.radians(15.0):
            steer_max_rate = math.radians(12.0)
        elif steer_abs > math.radians(8.0):
            steer_max_rate = math.radians(6.0)
        else:
            steer_max_rate = math.radians(3.0)
        steer_diff = steer - self._steer_prev
        if abs(steer_diff) > steer_max_rate:
            steer = self._steer_prev + steer_max_rate * (1 if steer_diff > 0 else -1)
        self._steer_prev = steer

        # ── Step 7: 速度制御（コーナーで自動減速）────────────────
        # front距離に応じて目標速度を動的に下げる
        if front_min < 0.8:
            speed_limit = self._speed_min              # 緊急: 最低速
        elif front_min < 1.2:
            speed_limit = self._speed_min + 0.1        # コーナー深部
        elif front_min < 1.5:
            speed_limit = self._speed_target * 0.6     # コーナー手前
        else:
            speed_limit = self._speed_target           # 直線: フル速度

        # コーナー突入時（speed_limitが下がった瞬間）PIDをリセット
        prev_limit = getattr(self, '_prev_speed_limit', speed_limit)
        if speed_limit < prev_limit - 0.05:
            self._pid.reset()
        self._prev_speed_limit = speed_limit

        if self._use_speed_pid:
            error = speed_limit - self._v_current
            speed = self._pid.compute(error, dt)
            speed = float(np.clip(speed, self._speed_min, speed_limit))
        else:
            speed = speed_limit

        self._publish_drive(steer, speed)

        # ── Step 8: 可視化マーカー ───────────────────────────────
        self._publish_markers(angles, proc, closest_idx,
                               gap_start, gap_end, alpha, target_dist)

        # ── ログ出力（CHANGE / SUMMARY）──────────────────────────
        steer_deg = math.degrees(steer)
        alpha_deg = math.degrees(alpha)
        gap_w_deg = math.degrees((gap_end - gap_start + 1) * angle_inc)
        log_line = (
            f'[{mode}] front={front_min:.2f}m closest={closest_dist:.2f}m '
            f'gap_w={gap_w_deg:.0f}deg alpha={alpha_deg:.1f}deg '
            f'steer={steer_deg:.1f}deg v={self._v_current:.2f}m/s spd={speed:.2f}'
        )
        prev = self._log_prev
        changed = (
            abs(steer_deg - prev.get('steer', steer_deg+999)) > self._LOG_STEER_TH or
            abs(front_min - prev.get('front', front_min+999)) > self._LOG_FRONT_TH or
            abs(gap_w_deg - prev.get('gap_w', gap_w_deg+999)) > self._LOG_GAP_W_TH or
            prev.get('mode','') != mode
        )
        if changed:
            self.get_logger().info(f'[CHANGE]  {log_line}')
            self._log_prev = {'steer': steer_deg, 'front': front_min,
                              'gap_w': gap_w_deg, 'mode': mode}
        if now - self._log_last_summary >= self._LOG_SUMMARY_INTERVAL:
            self.get_logger().info(f'[SUMMARY] {log_line}')
            self._log_last_summary = now

    def _publish_markers(self, angles, proc, closest_idx, gap_start, gap_end, alpha, target_dist):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        LIFETIME_NS = 300_000_000

        c_dist  = float(proc[closest_idx])
        c_angle = float(angles[closest_idx])
        m_bubble = Marker()
        m_bubble.header.frame_id = 'laser'
        m_bubble.header.stamp    = now
        m_bubble.ns='ftg'; m_bubble.id=0
        m_bubble.type   = Marker.CYLINDER
        m_bubble.action = Marker.ADD
        m_bubble.pose.position.x = -c_dist * math.cos(c_angle)
        m_bubble.pose.position.y = -c_dist * math.sin(c_angle)
        m_bubble.pose.position.z = 0.0
        m_bubble.pose.orientation.w = 1.0
        m_bubble.scale.x = self._bubble_radius * 2.0
        m_bubble.scale.y = self._bubble_radius * 2.0
        m_bubble.scale.z = 0.05
        m_bubble.color.r=1.0; m_bubble.color.g=0.4; m_bubble.color.b=0.0; m_bubble.color.a=0.45
        m_bubble.lifetime.nanosec = LIFETIME_NS
        marker_array.markers.append(m_bubble)

        m_gap = Marker()
        m_gap.header.frame_id='laser'; m_gap.header.stamp=now
        m_gap.ns='ftg'; m_gap.id=1
        m_gap.type=Marker.POINTS; m_gap.action=Marker.ADD
        m_gap.scale.x=0.04; m_gap.scale.y=0.04
        m_gap.color.r=0.0; m_gap.color.g=1.0; m_gap.color.b=1.0; m_gap.color.a=0.9
        m_gap.lifetime.nanosec = LIFETIME_NS
        for i in range(gap_start, gap_end+1):
            if proc[i] > 0:
                p=Point()
                p.x=-float(proc[i])*math.cos(float(angles[i]))
                p.y=-float(proc[i])*math.sin(float(angles[i]))
                p.z=0.0
                m_gap.points.append(p)
        marker_array.markers.append(m_gap)

        m_arrow = Marker()
        m_arrow.header.frame_id='laser'; m_arrow.header.stamp=now
        m_arrow.ns='ftg'; m_arrow.id=2
        m_arrow.type=Marker.ARROW; m_arrow.action=Marker.ADD
        start=Point(); start.x=0.0; start.y=0.0; start.z=0.0
        goal=Point()
        goal.x=-float(target_dist*math.cos(alpha))
        goal.y=-float(target_dist*math.sin(alpha))
        goal.z=0.0
        m_arrow.points=[start,goal]
        m_arrow.scale.x=0.02; m_arrow.scale.y=0.03; m_arrow.scale.z=0.03
        m_arrow.color.r=1.0; m_arrow.color.g=1.0; m_arrow.color.b=0.0; m_arrow.color.a=1.0
        m_arrow.lifetime.nanosec = LIFETIME_NS
        marker_array.markers.append(m_arrow)

        self._marker_pub.publish(marker_array)

    def _find_max_gap(self, proc):
        best_start=best_end=None; best_len=0; cur_start=None
        for i,val in enumerate(proc):
            if val>0:
                if cur_start is None: cur_start=i
            else:
                if cur_start is not None:
                    length=i-cur_start
                    if length>best_len:
                        best_len=length; best_start=cur_start; best_end=i-1
                    cur_start=None
        if cur_start is not None:
            length=len(proc)-cur_start
            if length>best_len:
                best_start=cur_start; best_end=len(proc)-1
        return best_start, best_end

    def _publish_drive(self, steer, speed):
        msg=AckermannDriveStamped()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id='base_link'
        msg.drive.steering_angle=steer
        msg.drive.speed=speed
        self._drive_pub.publish(msg)

    def _publish_stop(self):
        self._publish_drive(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node=FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
