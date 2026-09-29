#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import time
import threading
from trajectory_msgs.msg import JointTrajectory
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from aero_open_sdk.aero_hand import AeroHand
import serial.tools.list_ports

# 位置制御コマンドのコード
CTRL_POS = 0x11

class AeroHandInterpolatorNode:
    def __init__(self):
        rospy.init_node('aero_hand_interpolator_node')

        # 空文字にすると VID/PID による自動検出にフォールバックする
        self.port = rospy.get_param('~port', '/dev/hand')

        # シリアル通信エラーがこの回数連続したら切断とみなして再接続する
        self.max_consecutive_errors = rospy.get_param('~max_consecutive_errors', 20)

        self.hand = None
        self.connected_port = None
        self.error_count = 0

        # 内部状態管理用の変数
        self.current_values = [0.0] * 7  # 現在の出力値（最初は全開）
        self.start_values   = [0.0] * 7  # 補間開始時の値
        self.target_values  = [0.0] * 7  # 目標値

        self.start_time = 0.0
        self.duration = 0.0
        self.is_moving = False
        self.torque_enabled = False

        # スレッドセーフのためのロック
        self.lock = threading.Lock()

        # 初回接続（デバイスが挿されるまで待つ）
        self.connect()

        # Goal（目標角と遷移時間）を受け取るSubscriber
        self.sub = rospy.Subscriber('/aero_hand/command', JointTrajectory, self.trajectory_callback)
        self.srv = rospy.Service('~set_torque_enable', SetBool, self.torque_enable_callback)

        # タクタイルセンサー値のPublisher
        self.tactile_pub = rospy.Publisher('/aero_hand/tactile', Int32MultiArray, queue_size=10)

        # --- サーボ状態（位置・速度・電流・温度）の定期取得 ---
        # 温度・電流は他ノード（過熱時の安全停止、トルク相当値の記録など）が使えるよう
        # 常時 /aero_hand/temperature, /aero_hand/current にpublishする。
        self.temperature_pub = rospy.Publisher(
            '/aero_hand/temperature', Float32MultiArray, queue_size=1
        )
        self.current_pub = rospy.Publisher(
            '/aero_hand/current', Float32MultiArray, queue_size=1
        )

        self.status_poll_rate = rospy.get_param('~status_poll_rate', 1.0)  # Hz
        if self.status_poll_rate > 0:
            self.status_timer = rospy.Timer(
                rospy.Duration(1.0 / self.status_poll_rate), self.poll_status
            )

        # 切断検知・再接続を行うスレッド
        self.monitor_thread = threading.Thread(target=self.connection_monitor)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

        # 100Hz (10ms周期) でESP32への送信・補間計算を行うタイマー
        self.loop_rate = 100
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.loop_rate), self.update_and_send)

        rospy.loginfo("AeroHand 補間制御ノードが正常に起動しました。")
        rospy.loginfo("Topic: /aero_hand/command を待機中...")

    def connect(self):
        """ デバイスを待って接続し、Homingまで行う """
        while not rospy.is_shutdown():
            port = self.wait_for_port()
            rospy.loginfo("AeroHandを初期化中... (port: {})".format(port))
            try:
                hand = AeroHand(port=port)
                break
            except Exception as e:
                rospy.logwarn("シリアルポートを開けませんでした: {}".format(e))
                time.sleep(1.0)
        else:
            raise rospy.ROSInterruptException("接続中にシャットダウンされました")

        hand.ser.flush = lambda: None
        rospy.loginfo("ESP32の起動・初期リセットを待っています (2秒)...")
        time.sleep(2.0)

        rospy.loginfo("Homingを開始します。すべての指を手動で全開にしておいてください...")
        success = False
        # 最大3回までリトライを試みる
        for attempt in range(1, 4):
            try:
                if attempt > 1:
                    rospy.loginfo(f"Homingを再試行中... (試行 {attempt}/3)")

                # 初回はゴミデータに巻き込まれる仕様のため、
                # タイムアウトを1.0秒と短くして、素早く例外を発生させます
                hand.send_homing(timeout_s=4.0)

                success = True
                break  # 成功したらループを抜ける

            except Exception as e:
                # 1回目の失敗時は警告ログを出し、一瞬待ってからリトライ
                rospy.logwarn(f"Homing試行 {attempt} 回目がタイムアウトしました（初回の通信アライメントズレを検知）。")
                time.sleep(1.0)  # ESP32側がバッファの吸い上げを終えるための微小なウェイト

        if success:
            rospy.loginfo("Homing完了！現在の位置を 0.0 として登録しました。")
        else:
            rospy.logerr("Homingに完全に失敗しました。ケーブル等の接続を確認してください。")

        with self.lock:
            # Homing後は全開(0.0)から再スタート
            self.current_values = [0.0] * 7
            self.start_values   = [0.0] * 7
            self.target_values  = [0.0] * 7
            self.is_moving = False
            self.torque_enabled = False
            self.error_count = 0
            self.connected_port = port
            self.hand = hand

    def disconnect(self):
        """ シリアルポートを閉じて未接続状態にする """
        with self.lock:
            if self.hand is not None:
                try:
                    self.hand.ser.close()
                except Exception:
                    pass
            self.hand = None
            self.is_moving = False
            self.torque_enabled = False

    def connection_monitor(self):
        """ 切断を検知したら再接続する """
        while not rospy.is_shutdown():
            if self.hand is not None:
                if not os.path.exists(self.connected_port):
                    rospy.logwarn("デバイス ({}) が切断されました。再接続を待ちます...".format(self.connected_port))
                    self.disconnect()
                elif self.error_count >= self.max_consecutive_errors:
                    rospy.logwarn("シリアル通信エラーが {} 回連続しました。再接続します...".format(self.error_count))
                    self.disconnect()
                    time.sleep(1.0)

            if self.hand is None:
                try:
                    self.connect()
                    rospy.loginfo("AeroHandに再接続しました。")
                except rospy.ROSInterruptException:
                    return
            time.sleep(0.5)

    def wait_for_port(self):
        """ デバイスが接続されるまで待機し、見つかったポートを返す """
        while not rospy.is_shutdown():
            port = self.port if self.port else self.detect_port()
            if port and os.path.exists(port):
                return port
            rospy.loginfo_throttle(5, "デバイス ({}) の接続を待っています...".format(
                self.port if self.port else "VID:PID=303a:1001"))
            time.sleep(0.5)
        raise rospy.ROSInterruptException("デバイス待機中にシャットダウンされました")

    def detect_port(self):
        vendor_id = 0x303a
        product_id = 0x1001
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == vendor_id and port.pid == product_id:
                return port.device
        return None

    def torque_enable_callback(self, req):
        with self.lock:
            if self.hand is None:
                return SetBoolResponse(success=False, message="AeroHand is not connected")
            try:
                if req.data:
                    self.hand.set_torque_enable(True)
                    self.torque_enabled = True
                    rospy.loginfo("サーボのトルクをONにしました。")
                    return SetBoolResponse(success=True, message="Torque ENABLED successfully")
                else:
                    self.hand.set_torque_enable(False)
                    self.torque_enabled = False
                    self.is_moving = False # 移動中なら補間をストップ
                    rospy.loginfo("サーボのトルクをOFF（脱力）にしました。")
                    return SetBoolResponse(success=True, message="Torque DISABLED successfully")
            except Exception as e:
                rospy.logerr("トルク切り替えエラー: {}".format(e))
                return SetBoolResponse(success=False, message=str(e))

    def trajectory_callback(self, msg):
        print(msg)
        """ 目標角と時間を受け取るコールバック関数 """
        if not msg.points:
            rospy.logwarn("受け取ったメッセージにポイントデータが含まれていません。")
            return

        if self.hand is None:
            rospy.logwarn("AeroHandが未接続のため、目標を無視します。")
            return

        point = msg.points[0]
        if len(point.positions) != 7:
            rospy.logwarn("目標角度の要素数が7ではありません。無視します。")
            return

        # 遷移時間（秒）を取得
        tgt_duration = point.time_from_start.to_sec()
        if tgt_duration <= 0:
            tgt_duration = 0.01  # 0以下の場合は即座に遷移させる

        with self.lock:
            # 現在の位置を始点として、新しい目標へ滑らかに上書き補間をかける
            self.start_values = list(self.current_values)
            self.target_values = list(point.positions)
            self.start_time = time.time()
            self.duration = tgt_duration
            self.is_moving = True

        rospy.loginfo("新しい目標を受信: {} を {} 秒かけて動かします。".format(self.target_values, tgt_duration))

    def poll_status(self, event):
        """ 低頻度でサーボの電流・温度を取得し、
            /aero_hand/temperature, /aero_hand/current にpublishする。 """
        with self.lock:
            if self.hand is None:
                return
            try:
                currents = self.hand.get_actuator_currents()
                temperatures = self.hand.get_actuator_temperatures()
            except Exception as e:
                rospy.logwarn_throttle(10, "サーボ状態の取得エラー: {}".format(e))
                return

        if temperatures is not None:
            temp_msg = Float32MultiArray()
            temp_msg.data = temperatures
            self.temperature_pub.publish(temp_msg)

        if currents is not None:
            current_msg = Float32MultiArray()
            current_msg.data = currents
            self.current_pub.publish(current_msg)

    def update_and_send(self, event):
        """ 100Hzでバックグラウンド実行される計算・送信ループ """
        # --- タクタイルセンサー値の取得とPublish ---
        # (log_status など他スレッドとシリアルポートを取り合わないようlockで保護)
        with self.lock:
            if self.hand is None:
                return
            try:
                tactile_data = self.hand.get_tactile_data()
                self.error_count = 0
            except Exception as e:
                tactile_data = None
                self.error_count += 1
        if tactile_data is not None:
            tactile_msg = Int32MultiArray()
            tactile_msg.data = tactile_data
            self.tactile_pub.publish(tactile_msg)

        if not self.is_moving:
            return
        with self.lock:
            if self.hand is None:
                return
            if self.is_moving:
                now = time.time()
                elapsed = now - self.start_time

                if elapsed >= self.duration:
                    # 目標時間に到達、または超過した場合は目標値で固定
                    self.current_values = list(self.target_values)
                    self.is_moving = False
                else:
                    # 線形補間 (Linear Interpolation) の計算
                    t = elapsed / self.duration
                    self.current_values = [
                        self.start_values[i] + t * (self.target_values[i] - self.start_values[i])
                        for i in range(7)
                    ]

            # ご提示いただいた数式で 0.0〜1.0 を 0〜65535 の uint16 データに変換
            payload = [int(max(0.0, min(1.0, v)) * 65535) for v in self.current_values]

            try:
                # ESP32へダイレクト送信
                self.hand._send_data(CTRL_POS, payload)
            except Exception as e:
                self.error_count += 1
                rospy.logerr_throttle(1, "ESP32への送信エラー: {}".format(e))

if __name__ == '__main__':
    try:
        node = AeroHandInterpolatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
