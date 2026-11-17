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

引いた方向は前後左右の並進に使う (~allow_lateral, 既定 True).
回転は画像認識で求めた人の体の向き (~body_yaw_topic, aero_demo の
human_body_yaw_node.py が出す geometry_msgs/PoseStamped) に台車の向きを揃える
(~enable_body_rotation):
- 向きの差が rot_start_angle を超えた状態が rot_start_delay 秒続いたら回り始め,
  rot_stop_angle を下回ったら止まる (ヒステリシス).
- 角速度は 向きの差 x rot_gain (上限 max_rot_vel), 角加速度は max_rot_accel で制限する.
- 回っている間も引きによる並進は止めない. 回転の中心は ~rotation_center
  ('hand': つないでいる手 (~frame_id), 'base': 台車の中心). 'hand' のときは
  v = -omega x r の並進を足して, 回っても手の位置が動かないようにする.
- 向きが body_yaw_timeout 秒届かなければ回転だけ 0 にする.
向きのメッセージは撮影時刻の odom 座標系で届くので, 最新の TF で base_link に直して
使う (画像の遅れの間に台車が回っても回りすぎない).
"""

import math
import threading

import numpy as np
import rospy
import serial
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float32
from tf.transformations import quaternion_matrix


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    """クォータニオン (x, y, z, w を持つもの) のヨー角."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rotation_compensation(omega, r):
    """台車の中心から r (xy) にある点を中心に omega で回るための並進 v = -omega x r."""
    return np.array([omega * r[1], -omega * r[0]])


class BodyYawRotation:
    """人の体の向きとの差 (向きの差) から台車の角速度を決める (ROS 非依存).

    向きの差が start_angle を超えた状態が start_delay 秒続いたら回り始め,
    stop_angle を下回ったら止まる. 角速度は gain x 向きの差 (上限 max_vel),
    変化は max_accel [rad/s^2] で制限する.
    """

    def __init__(self, start_angle, stop_angle, start_delay, gain, max_vel, max_accel):
        self.start_angle = start_angle
        self.stop_angle = min(stop_angle, start_angle)
        self.start_delay = start_delay
        self.gain = gain
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.reset()

    def reset(self):
        self.rotating = False
        self.over_since = None
        self.omega = 0.0

    def update(self, error, now, dt):
        """error: 向きの差 [rad] (base_link から見た人の向き, 無ければ None), now: 時刻 [s]."""
        target = 0.0
        if error is None:
            self.rotating = False
            self.over_since = None
        else:
            a = abs(error)
            if self.rotating:
                if a < self.stop_angle:
                    self.rotating = False
                    self.over_since = None
            elif a > self.start_angle:
                if self.over_since is None:
                    self.over_since = now
                if now - self.over_since >= self.start_delay:
                    self.rotating = True
            else:
                self.over_since = None
            if self.rotating:
                target = float(np.clip(self.gain * error, -self.max_vel, self.max_vel))
        step = self.max_accel * dt
        self.omega += float(np.clip(target - self.omega, -step, step))
        return self.omega


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
        # 11/17: 回転だけを試すため既定は 0 (引きで動かない). 引きで動かすなら 0.4
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.0)  # m/s
        # 全方向台車なら True (横に引くと横へ動く). False なら横に引くと旋回する
        self.allow_lateral = rospy.get_param('~allow_lateral', True)
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

        # 人の体の向きに台車の向きを揃える回転
        self.enable_body_rotation = rospy.get_param('~enable_body_rotation', True)
        self.body_yaw_topic = rospy.get_param('~body_yaw_topic', '/human_body_yaw/body_pose')
        self.body_yaw_timeout = rospy.get_param('~body_yaw_timeout', 0.5)  # s (受信からの経過)
        self.rotation_center = rospy.get_param('~rotation_center', 'base')  # 'hand' or 'base'
        # 並進 (引き + 回転の補正) の合計の上限
        self.max_total_linear_vel = rospy.get_param('~max_total_linear_vel', 0.5)  # m/s
        # 既定は 11/17 の実機試行で使った値
        self.rotation = BodyYawRotation(
            start_angle=math.radians(rospy.get_param('~rot_start_angle', 8.0)),
            stop_angle=math.radians(rospy.get_param('~rot_stop_angle', 3.0)),
            start_delay=rospy.get_param('~rot_start_delay', 0.1),
            gain=rospy.get_param('~rot_gain', 1.5),                  # (rad/s) / rad
            max_vel=rospy.get_param('~max_rot_vel', 0.5),            # rad/s
            max_accel=rospy.get_param('~max_rot_accel', 1.2))        # rad/s^2
        if self.enable_body_rotation and not self.allow_lateral:
            rospy.logwarn("enable_body_rotation needs allow_lateral=True "
                          "(the lateral pull is used for turning). Body rotation disabled.")
            self.enable_body_rotation = False
        if self.enable_body_rotation and not self.use_tf:
            rospy.logwarn("enable_body_rotation needs use_tf=True. Body rotation disabled.")
            self.enable_body_rotation = False
        if self.rotation_center not in ('hand', 'base'):
            rospy.logwarn(f"Unknown rotation_center '{self.rotation_center}', using 'hand'.")
            self.rotation_center = 'hand'
        self.body_yaw_lock = threading.Lock()
        self.body_yaw_msg = None
        self.body_yaw_recv = None  # 受信時刻 (PC とロボットの時計のずれに左右されないよう受信側の時計)

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
        # デバッグ用: 向きの差 [deg] (人の向き - 台車の向き) と回転中か
        self.yaw_error_pub = rospy.Publisher('~body_yaw_error', Float32, queue_size=1)
        self.rotating_pub = rospy.Publisher('~rotating', Bool, queue_size=1)
        if self.enable_body_rotation:
            rospy.Subscriber(self.body_yaw_topic, PoseStamped, self.body_yaw_callback, queue_size=1)

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
                      f"use_tf: {self.use_tf}, base_frame: {self.base_frame}, "
                      f"allow_lateral: {self.allow_lateral}")
        if self.enable_body_rotation:
            r = self.rotation
            rospy.loginfo(f"Body rotation -> topic: {self.body_yaw_topic}, "
                          f"start/stop: {math.degrees(r.start_angle):.0f}/{math.degrees(r.stop_angle):.0f} deg "
                          f"after {r.start_delay} s, gain: {r.gain}, max: {r.max_vel} rad/s, "
                          f"accel: {r.max_accel} rad/s^2, center: {self.rotation_center}")

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

    # ---------------- 人の体の向き ----------------
    def body_yaw_callback(self, msg):
        with self.body_yaw_lock:
            self.body_yaw_msg = msg
            self.body_yaw_recv = rospy.Time.now()

    def lookup_base_from(self, frame):
        """base_frame <- frame の最新の TF (取れなければ None)."""
        try:
            return self.tf_buffer.lookup_transform(self.base_frame, frame, rospy.Time(0),
                                                   rospy.Duration(0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, f"TF {frame} -> {self.base_frame} failed: {e}")
            return None

    def body_yaw_error(self):
        """base_frame から見た人の体の向き [rad] (= 台車が回るべき角度). 無ければ None."""
        with self.body_yaw_lock:
            msg = self.body_yaw_msg
            recv = self.body_yaw_recv
        if msg is None or recv is None:
            return None
        if (rospy.Time.now() - recv).to_sec() > self.body_yaw_timeout:
            rospy.logwarn_throttle(5.0, "Body yaw timed out. Not rotating.")
            return None
        yaw = yaw_from_quaternion(msg.pose.orientation)
        if msg.header.frame_id and msg.header.frame_id != self.base_frame:
            t = self.lookup_base_from(msg.header.frame_id)
            if t is None:
                return None
            yaw += yaw_from_quaternion(t.transform.rotation)
        return wrap_angle(yaw)

    def rotation_center_xy(self):
        """回転の中心 (base_frame の xy). 取れなければ None."""
        if self.rotation_center == 'base':
            return np.zeros(2)
        t = self.lookup_base_from(self.hand_frame)
        if t is None:
            return None
        return np.array([t.transform.translation.x, t.transform.translation.y])

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

        dt = 1.0 / self.rate_hz
        self.filtered_cmd = self.rate_limit(target, dt)

        omega, linear = self.body_rotation(dt)
        linear = linear + self.filtered_cmd

        # オンの間は止まっているとき (不感帯内, キャリブレーション中, TF 取得失敗,
        # センサ途絶) も含めて常に publish する
        self.cmd_pub.publish(self.make_twist(linear, omega))

    def body_rotation(self, dt):
        """人の体の向きに揃える角速度と, 回転の中心を保つための並進 (base_frame の xy)."""
        if not self.enable_body_rotation:
            return 0.0, np.zeros(2)
        error = self.body_yaw_error()
        center = self.rotation_center_xy() if error is not None else None
        if center is None:
            error = None  # 手の位置が分からなければ手を中心に回せないので回さない
        omega = self.rotation.update(error, rospy.Time.now().to_sec(), dt)
        if error is not None:
            self.yaw_error_pub.publish(Float32(data=math.degrees(error)))
        self.rotating_pub.publish(Bool(data=self.rotation.rotating))
        if center is None:
            # 止まるまでの減速中は台車の中心で回る
            return omega, np.zeros(2)
        return omega, rotation_compensation(omega, center)

    def make_twist(self, v, omega=0.0):
        twist = Twist()
        if self.allow_lateral:
            n = np.linalg.norm(v)
            if n > self.max_total_linear_vel > 0:
                v = v * (self.max_total_linear_vel / n)
            twist.linear.x = v[0]
            twist.linear.y = v[1]
            twist.angular.z = omega
        else:
            # 差動二輪: 前後成分で並進, 横成分で旋回
            twist.linear.x = v[0]
            if self.max_linear_vel > 0:
                twist.angular.z = (v[1] / self.max_linear_vel) * self.max_angular_vel
        return twist

    def stop(self):
        self.filtered_cmd = np.zeros(2)
        self.pulling = False
        self.rotation.reset()
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
