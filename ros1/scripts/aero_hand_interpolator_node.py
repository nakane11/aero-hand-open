#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import threading
from trajectory_msgs.msg import JointTrajectory
from aero_open_sdk.aero_hand import AeroHand
import serial.tools.list_ports

# 位置制御コマンドのコード
CTRL_POS = 0x11

class AeroHandInterpolatorNode:
    def __init__(self):
        rospy.init_node('aero_hand_interpolator_node')

        rospy.loginfo("AeroHandを初期化中...")
        self.hand = AeroHand(port=self.detect_port())
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
                self.hand.send_homing(timeout_s=4.0)

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

        # 内部状態管理用の変数
        self.current_values = [0.0] * 7  # 現在の出力値（最初は全開）
        self.start_values   = [0.0] * 7  # 補間開始時の値
        self.target_values  = [0.0] * 7  # 目標値

        self.start_time = 0.0
        self.duration = 0.0
        self.is_moving = False

        # スレッドセーフのためのロック
        self.lock = threading.Lock()

        # 100Hz (10ms周期) でESP32への送信・補間計算を行うタイマー
        self.loop_rate = 100
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.loop_rate), self.update_and_send)

        # Goal（目標角と遷移時間）を受け取るSubscriber
        self.sub = rospy.Subscriber('/aero_hand/command', JointTrajectory, self.trajectory_callback)

        rospy.loginfo("AeroHand 補間制御ノードが正常に起動しました。")
        rospy.loginfo("Topic: /aero_hand/command を待機中...")

    def detect_port(self):
        vendor_id = 0x303a
        product_id = 0x1001
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == vendor_id and port.pid == product_id:
                return port.device
        return None

    def trajectory_callback(self, msg):
        """ 目標角と時間を受け取るコールバック関数 """
        if not msg.points:
            rospy.logwarn("受け取ったメッセージにポイントデータが含まれていません。")
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

    def update_and_send(self, event):
        """ 100Hzでバックグラウンド実行される計算・送信ループ """
        with self.lock:
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
                rospy.logerr("ESP32への送信エラー: {}".format(e))

if __name__ == '__main__':
    try:
        node = AeroHandInterpolatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
