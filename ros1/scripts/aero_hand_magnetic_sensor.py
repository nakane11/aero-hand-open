#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import math
import numpy as np

from geometry_msgs.msg import Vector3Stamped, Point
from visualization_msgs.msg import Marker

class MagneticSensorNode:
    def __init__(self):
        rospy.init_node('aero_hand_magnetic_sensor')

        # パラメータ設定
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.frame_id = rospy.get_param('~frame_id', 'r_hand_link')
        self.angle_threshold = rospy.get_param('~angle_threshold', 15.0) # deg

        # 符号反転用フラグ
        self.invert_rot_x = False  # X軸周り回転の正負反転
        self.invert_rot_y = True   # Y軸周り回転の正負反転（vector.yの正負を反転）

        # 状態変数
        self.initial_vector = None
        self.calibration_samples = []
        self.local_R = None

        # パブリッシャー
        self.tilt_pub = rospy.Publisher('/magnetic_sensor/tilt_vector', Vector3Stamped, queue_size=10)
        self.marker_pub = rospy.Publisher('/magnetic_sensor/tilt_marker', Marker, queue_size=10)

        # シリアル接続
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            rospy.loginfo(f"Connected to {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            return

        self.read_thread = threading.Thread(target=self.serial_read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

        rospy.loginfo("Magnetic sensor node started.")

    def setup_coordinate_system(self, v_init):
        z_axis = v_init / np.linalg.norm(v_init)
        ref_axis = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x_axis = np.cross(ref_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        self.local_R = np.vstack([x_axis, y_axis, z_axis])

    def calculate_angles(self, current_vector):
        v_curr = np.array(current_vector)
        v_local = np.dot(self.local_R, v_curr)
        x, y, z = v_local[0], v_local[1], abs(v_local[2])

        # 生の回転角度の抽出
        raw_rot_x = math.atan2(x, z)  # X軸周り回転
        raw_rot_y = math.atan2(y, z)  # Y軸周り回転

        # パブリッシュ用角度（符号調整適用）
        pub_rot_x = -raw_rot_x if self.invert_rot_x else raw_rot_x
        pub_rot_y = -raw_rot_y if self.invert_rot_y else raw_rot_y

        r_xy = math.sqrt(x**2 + y**2)
        total_theta_rad = math.atan2(r_xy, z)

        # (パブリッシュ用rot_x, パブリッシュ用rot_y, RViz描画用raw_rot_y, raw_rot_x, 総合傾き)
        return pub_rot_x, pub_rot_y, raw_rot_x, raw_rot_y, total_theta_rad

    def publish_rviz_marker(self, raw_rot_x_rad, raw_rot_y_rad, total_theta_rad):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "magnetic_tilt"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        start_point = Point(0.0, 0.0, 0.0)
        arrow_length = 0.15
        z_direction = -1.0  # 指先方向 (-Z)

        # RViz描画用（正しい表示を維持する元の符号位置を参照）
        end_x = arrow_length * math.sin(raw_rot_y_rad)
        end_y = arrow_length * math.sin(raw_rot_x_rad)
        end_z = z_direction * arrow_length * math.cos(total_theta_rad)

        marker.points = [start_point, Point(end_x, end_y, end_z)]

        marker.scale.x = 0.008
        marker.scale.y = 0.016
        marker.scale.z = 0.025

        tilt_deg = math.degrees(total_theta_rad)
        norm_tilt = min(tilt_deg / self.angle_threshold, 1.0)
        marker.color.r = norm_tilt
        marker.color.g = 1.0 - norm_tilt
        marker.color.b = 0.2
        marker.color.a = 0.9

        self.marker_pub.publish(marker)

    def serial_read_loop(self):
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line or "ERROR" in line:
                    continue

                parts = line.split(',')
                if len(parts) == 3:
                    x, y, z = map(float, parts)
                    current_vector = (x, y, z)

                    if self.initial_vector is None:
                        self.calibration_samples.append(current_vector)
                        if len(self.calibration_samples) < 50:
                            continue

                        avg_x = sum(v[0] for v in self.calibration_samples) / 50.0
                        avg_y = sum(v[1] for v in self.calibration_samples) / 50.0
                        avg_z = sum(v[2] for v in self.calibration_samples) / 50.0
            
                        self.initial_vector = np.array([avg_x, avg_y, avg_z])
                        self.setup_coordinate_system(self.initial_vector)
                        rospy.loginfo(f"Calibration complete: {self.initial_vector}")
                        continue

                    pub_rot_x, pub_rot_y, raw_rot_x, raw_rot_y, total_rad = self.calculate_angles(current_vector)

                    # 数値トピックへのパブリッシュ
                    tilt_msg = Vector3Stamped()
                    tilt_msg.header.stamp = rospy.Time.now()
                    tilt_msg.header.frame_id = self.frame_id
                    tilt_msg.vector.x = math.degrees(pub_rot_x)  # X軸周り回転角 [deg]
                    tilt_msg.vector.y = math.degrees(pub_rot_y)  # Y軸周り回転角 [deg] (符号反転済み)
                    tilt_msg.vector.z = math.degrees(total_rad)  # 総合傾き角 [deg]
                    self.tilt_pub.publish(tilt_msg)

                    # RViz表示用マーカーのパブリッシュ（正確な向きを保持）
                    self.publish_rviz_marker(raw_rot_x, raw_rot_y, total_rad)

            except ValueError:
                pass
            except serial.SerialException as e:
                rospy.logerr(f"Serial read error: {e}")
                rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        node = MagneticSensorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
