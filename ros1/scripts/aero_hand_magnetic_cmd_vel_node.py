#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手首を引いた方向にロボット（台車）を動かすノード.

磁気センサ (firmware/wrist) をシリアルから直接読み, aero_hand_magnetic_sensor.py と
同じ方法で傾き方向を求め, ハンド座標系 -> ベース座標系に TF で変換して
xy 平面に射影した方向へ cmd_vel を送る.

オン/オフは同じ AtomS3 のボタンで切り替える. ファームウェアが同じシリアルに
"BTN,1" / "BTN,0" を送ってくるので, その状態が変わったときに開始/停止する.
~enable (std_msgs/Bool) トピックからも切り替えられる.
オンにするたびにキャリブレーション (最初の 50 サンプル) をやり直す.

傾き角が dead_zone を超えたら動き出し, dead_zone_release を下回るまで動き続ける
(ヒステリシス). max_tilt で最大速度に達する.
前後 (base_frame の x) 方向の速度だけ gain_x 倍する (手首が前後には傾きにくいため).
引いた方向が前後左右の軸から snap_angle 以内なら, その軸方向だけに揃える.
速度の変化は max_accel (加速) / max_decel (減速) で制限する.
オンの間は 20Hz (~rate) で常に cmd_vel を送る (止まっているときは 0).
センサ途絶 (timeout) や TF 取得失敗のときは 0 を送り, オフにしたら 0 を 1 回送って以降は送らない.
BTN 行が btn_timeout 秒届かなければ (USB 抜け, ファームウェア停止など) 自動でオフにする.
再びオンにするにはボタンを押し直す.
"""

import math
import threading

import numpy as np
import rospy
import serial
import tf2_ros
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf.transformations import quaternion_matrix


class MagneticCmdVelNode:
    def __init__(self):
        rospy.init_node('aero_hand_magnetic_cmd_vel')

        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.hand_frame = rospy.get_param('~frame_id', 'r_hand_link')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.use_tf = rospy.get_param('~use_tf', True)
        self.dead_zone = rospy.get_param('~dead_zone', 5.0)        # deg (動き出す傾き)
        # 動いている間に止まる傾き. ロボットが動くと手首の傾きが戻るので, 境目で
        # 出たり止まったりしないよう dead_zone より小さくする
        self.dead_zone_release = min(rospy.get_param('~dead_zone_release', 3.0), self.dead_zone)  # deg
        self.max_tilt = rospy.get_param('~max_tilt', 20.0)         # deg
        # 前後 (base_frame の x) 方向だけに掛けるゲイン. 実機テストで手首が前後には
        # 左右より傾きにくく, 同じように引いても速度が左右の約 0.7 倍だったため 1.5 倍にする
        self.gain_x = rospy.get_param('~gain_x', 1.5)
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.4)  # m/s
        self.allow_lateral = rospy.get_param('~allow_lateral', False)  # 全方向台車なら True
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.7)  # rad/s (allow_lateral=False 時)
        self.direction_sign = rospy.get_param('~direction_sign', 1.0)  # 引く方向と逆に動く場合は -1
        self.snap_angle = rospy.get_param('~snap_angle', 20.0)     # deg (0 で無効)
        self.max_accel = rospy.get_param('~max_accel', 0.4)        # m/s^2 (加速側)
        # 減速側. 人の手の追従が遅れて引きが一瞬抜けても急に止まらないよう加速と同程度にする.
        # 緊急時はボタンでオフにすれば減速制限なしで即 0 になる
        self.max_decel = rospy.get_param('~max_decel', 0.4)        # m/s^2
        self.btn_timeout = rospy.get_param('~btn_timeout', 3.0)    # s (0 で無効)
        self.timeout = rospy.get_param('~timeout', 0.5)            # s
        self.rate_hz = rospy.get_param('~rate', 20.0)
        self.calibration_count = rospy.get_param('~calibration_count', 50)

        # use_tf=False の場合に使うハンド座標系 -> ベース座標系の軸マッピング
        # vx = map_x[0]*hand_x + map_x[1]*hand_y, vy = map_y[0]*hand_x + map_y[1]*hand_y
        self.map_x = rospy.get_param('~map_x', [1.0, 0.0])
        self.map_y = rospy.get_param('~map_y', [0.0, 1.0])

        self.lock = threading.Lock()
        self.toggle_lock = threading.Lock()
        self.latest_dir = None   # (np.array(3) ハンド座標系の方向, 傾き角[deg])
        self.latest_stamp = None
        self.filtered_cmd = np.zeros(2)
        self.pulling = False  # 不感帯ヒステリシスの状態

        self.initial_vector = None
        self.calibration_samples = []
        self.local_R = None

        self.last_fw_button = None  # ファームウェアから最後に受け取った BTN 状態
        self.last_btn_stamp = None  # 最後に BTN 行を受け取った時刻
        self.active = False

        if self.use_tf:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.active_pub = rospy.Publisher('~active', Bool, queue_size=1, latch=True)
        self.active_pub.publish(Bool(data=False))

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            rospy.loginfo(f"Connected to {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            return

        # ボタン以外からも切り替えられるように (True: オン, False: オフ)
        rospy.Subscriber('~enable', Bool, self.enable_callback, queue_size=1)

        self.read_thread = threading.Thread(target=self.serial_read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.timer_cb)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("Magnetic cmd_vel node started. Press the AtomS3 button to start/stop.")
        rospy.loginfo(f"Parameters -> dead_zone: {self.dead_zone}/{self.dead_zone_release} deg, max_tilt: {self.max_tilt} deg, "
                      f"max_vel: {self.max_linear_vel} m/s, snap: {self.snap_angle} deg, "
                      f"accel/decel: {self.max_accel}/{self.max_decel} m/s^2, btn_timeout: {self.btn_timeout} s, "
                      f"use_tf: {self.use_tf}, base_frame: {self.base_frame}")

    # ---------------- ボタンによるオン/オフ ----------------
    def fw_button_callback(self, state):
        """ファームウェアの "BTN,<0|1>" 行. 状態が変わったときだけ切り替える
        (1 秒ごとの定期送信では何もしないので, ~enable での切り替えは上書きされない)."""
        self.last_btn_stamp = rospy.Time.now()
        if state == self.last_fw_button:
            return
        self.last_fw_button = state
        with self.toggle_lock:
            if state and not self.active:
                self.start_feedback()
            elif not state and self.active:
                self.stop_feedback()

    def enable_callback(self, msg):
        with self.toggle_lock:
            if msg.data and not self.active:
                self.start_feedback()
            elif not msg.data and self.active:
                self.stop_feedback()

    def start_feedback(self):
        # オンにするたびにキャリブレーションをやり直す
        with self.lock:
            self.initial_vector = None
            self.calibration_samples = []
            self.local_R = None
            self.latest_dir = None
            self.latest_stamp = None
        self.active = True
        self.active_pub.publish(Bool(data=True))
        rospy.loginfo("cmd_vel control started (calibrating... keep the hand still).")

    def stop_feedback(self):
        self.active = False
        with self.lock:
            self.latest_dir = None
            self.latest_stamp = None
        self.stop()
        self.active_pub.publish(Bool(data=False))
        rospy.loginfo("cmd_vel control stopped.")

    # ---------------- センサ処理 ----------------
    def setup_coordinate_system(self, v_init):
        z_axis = v_init / np.linalg.norm(v_init)
        ref_axis = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x_axis = np.cross(ref_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        self.local_R = np.vstack([x_axis, y_axis, z_axis])

    def calculate_direction(self, current_vector):
        """aero_hand_magnetic_sensor.py の RViz マーカーと同じ方向をハンド座標系で返す."""
        v_local = np.dot(self.local_R, np.array(current_vector))
        x, y, z = v_local[0], v_local[1], abs(v_local[2])
        raw_rot_x = math.atan2(x, z)
        raw_rot_y = math.atan2(y, z)
        total_deg = math.degrees(math.atan2(math.sqrt(x ** 2 + y ** 2), z))
        d = np.array([math.sin(raw_rot_y), math.sin(raw_rot_x), 0.0])
        return d, total_deg

    def serial_read_loop(self):
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                if "ERROR" in line:
                    rospy.logwarn_throttle(5.0, f"Firmware: {line}")
                    continue

                parts = line.split(',')
                if len(parts) == 2 and parts[0] == 'BTN':
                    self.fw_button_callback(parts[1].strip() == '1')
                    continue
                if len(parts) != 3 or not self.active:
                    continue
                current_vector = tuple(map(float, parts))

                with self.lock:
                    if self.initial_vector is None:
                        self.calibration_samples.append(current_vector)
                        if len(self.calibration_samples) < self.calibration_count:
                            continue
                        self.initial_vector = np.mean(np.array(self.calibration_samples), axis=0)
                        self.setup_coordinate_system(self.initial_vector)
                        rospy.loginfo(f"Calibration complete. Initial vector: {self.initial_vector}")
                        continue

                    d, tilt_deg = self.calculate_direction(current_vector)
                    self.latest_dir = (d, tilt_deg)
                    self.latest_stamp = rospy.Time.now()

            except ValueError:
                pass  # 途中で切れた行は無視
            except serial.SerialException as e:
                rospy.logerr(f"Serial read error: {e}")
                rospy.sleep(1.0)
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")

    # ---------------- cmd_vel 出力 ----------------
    def hand_to_base(self, d):
        """ハンド座標系のベクトルをベース座標系の xy 成分に変換."""
        if self.use_tf:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, self.hand_frame, rospy.Time(0),
                                                    rospy.Duration(0.05))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(2.0, f"TF {self.hand_frame} -> {self.base_frame} failed: {e}")
                return None
            q = t.transform.rotation
            R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            v = R.dot(d)
            return np.array([v[0], v[1]])
        return np.array([self.map_x[0] * d[0] + self.map_x[1] * d[1],
                         self.map_y[0] * d[0] + self.map_y[1] * d[1]])

    def snap_to_axis(self, v):
        """方向 v (ベース座標系 xy) が前後左右の軸から snap_angle 以内ならその軸に揃える."""
        if self.snap_angle <= 0.0:
            return v
        n = np.linalg.norm(v)
        ang = math.atan2(v[1], v[0])
        axis = round(ang / (math.pi / 2)) * (math.pi / 2)
        if abs(ang - axis) <= math.radians(self.snap_angle):
            return n * np.array([math.cos(axis), math.sin(axis)])
        return v

    def rate_limit(self, target, dt):
        """速度ベクトルの変化量を加速/減速の上限で制限する."""
        diff = target - self.filtered_cmd
        dn = np.linalg.norm(diff)
        if dn < 1e-9:
            return target
        accelerating = np.linalg.norm(target) > np.linalg.norm(self.filtered_cmd)
        max_step = (self.max_accel if accelerating else self.max_decel) * dt
        if dn <= max_step:
            return target
        return self.filtered_cmd + diff / dn * max_step

    def check_btn_watchdog(self):
        """BTN 行が btn_timeout 秒届かなければオフにする (一度も BTN を受けていなければ何もしない)."""
        if self.btn_timeout <= 0.0 or self.last_btn_stamp is None:
            return
        if (rospy.Time.now() - self.last_btn_stamp).to_sec() > self.btn_timeout:
            with self.toggle_lock:
                if self.active:
                    rospy.logwarn(f"No BTN message for {self.btn_timeout} s (USB disconnected?). Stopping.")
                    self.stop_feedback()

    def timer_cb(self, _event):
        if not self.active:
            return
        self.check_btn_watchdog()
        if not self.active:
            return

        with self.lock:
            latest = self.latest_dir
            stamp = self.latest_stamp

        target = np.zeros(2)
        if latest is not None and stamp is not None:
            if (rospy.Time.now() - stamp).to_sec() < self.timeout:
                d, tilt_deg = latest
                # 不感帯のヒステリシス: dead_zone を超えたら動き出し,
                # dead_zone_release を下回るまでは動き続ける
                if self.pulling:
                    self.pulling = tilt_deg > self.dead_zone_release
                else:
                    self.pulling = tilt_deg > self.dead_zone
                if self.pulling:
                    v = self.hand_to_base(d)
                    if v is not None:
                        n = np.linalg.norm(v)
                        if n > 1e-6:
                            # 動いている間は dead_zone_release を基準に速度を決める
                            # (動き出しの小さな段差は加速度制限でならされる)
                            ratio = min(max(tilt_deg - self.dead_zone_release, 0.0) /
                                        max(self.max_tilt - self.dead_zone_release, 1e-6), 1.0)
                            v = self.snap_to_axis(v / n)
                            target = self.direction_sign * v * ratio * self.max_linear_vel
                            # 前後だけゲインを掛ける (最大速度は超えない)
                            target[0] = float(np.clip(target[0] * self.gain_x,
                                                      -self.max_linear_vel, self.max_linear_vel))
            else:
                self.pulling = False
                rospy.logwarn_throttle(2.0, "Sensor data timed out. Stopping.")

        self.filtered_cmd = self.rate_limit(target, 1.0 / self.rate_hz)

        # オンの間は止まっているとき (不感帯内, キャリブレーション中, TF 取得失敗,
        # センサ途絶) も含めて常に publish する
        self.cmd_pub.publish(self.make_twist(self.filtered_cmd))

    def make_twist(self, v):
        twist = Twist()
        if self.allow_lateral:
            twist.linear.x = v[0]
            twist.linear.y = v[1]
        else:
            # 差動二輪: 前後成分で並進, 横成分で旋回
            twist.linear.x = v[0]
            if self.max_linear_vel > 0:
                twist.angular.z = (v[1] / self.max_linear_vel) * self.max_angular_vel
        return twist

    def stop(self):
        self.filtered_cmd = np.zeros(2)
        self.pulling = False
        self.cmd_pub.publish(Twist())

    def on_shutdown(self):
        self.active = False
        self.stop()


if __name__ == '__main__':
    try:
        MagneticCmdVelNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
