#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""/aero_hand_grasp_service/grasp (std_srvs/SetBool) に対して
data: false と data: true を interval 秒おきに交互に送信し続ける。

/aero_hand/temperature (Float32MultiArray, 7要素・度C) を監視し、いずれかの
サーボが --temp-threshold (デフォルト70度) 以上になったら、握る動作(data=true)を
止めて開いた状態(data=false)を維持する。全サーボが閾値を下回ったら通常の
トグル動作を再開する。

/aero_hand/current (Float32MultiArray, 7要素・mA) も併せて記録する。
サーボにトルクセンサーは無いため、実測トルクの代わりに実測電流(mA)を
「トルク相当値」として記録している点に注意。

/aero_hand/temperature, /aero_hand/current は aero_hand_interpolator_node が
publishしている。aero_hand.launch 起動中でないと値は流れてこない。

Usage
-----
    rosrun aero_demo toggle_grasp_service.py
    python3 toggle_grasp_service.py            # 直接実行も可
    python3 toggle_grasp_service.py --interval 3.0 --temp-threshold 65.0
"""

import argparse
import datetime
import json
import os

import rospy
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool

SERVICE_NAME = '/aero_hand_grasp_service/grasp'
TEMPERATURE_TOPIC = '/aero_hand/temperature'
CURRENT_TOPIC = '/aero_hand/current'


class ThermalAwareGraspToggler:
    def __init__(self, interval, temp_threshold, log_dir, log_enable):
        self.interval = interval
        self.temp_threshold = temp_threshold

        # /aero_hand/temperature, /aero_hand/current から届いた最新値
        self.latest_temperatures = None
        self.latest_currents = None

        self.log_file = None
        if log_enable:
            log_dir = os.path.expanduser(log_dir)
            os.makedirs(log_dir, exist_ok=True)
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            log_path = os.path.join(log_dir, f'toggle_grasp_{ts}.jsonl')
            self.log_file = open(log_path, 'a')
            rospy.loginfo('記録先: %s', log_path)
            rospy.on_shutdown(self._close_log)

        rospy.Subscriber(TEMPERATURE_TOPIC, Float32MultiArray, self._temperature_cb)
        rospy.Subscriber(CURRENT_TOPIC, Float32MultiArray, self._current_cb)

        rospy.loginfo('waiting for service %s ...', SERVICE_NAME)
        rospy.wait_for_service(SERVICE_NAME)
        self.grasp = rospy.ServiceProxy(SERVICE_NAME, SetBool)

    def _temperature_cb(self, msg):
        self.latest_temperatures = list(msg.data)

    def _current_cb(self, msg):
        self.latest_currents = list(msg.data)

    def _close_log(self):
        if self.log_file is not None:
            try:
                self.log_file.close()
            except Exception:
                pass

    def is_overheated(self):
        # 温度をまだ一度も受信していない間は false 扱い（起動直後の数秒だけ発生しうる）。
        # aero_hand.launch 側の火照りクランプ(main.cppのHOT_TORQUE_LIMIT)が
        # バックアップとして別途効くため、ここでは起動直後の一時的な未受信を
        # 過度に安全側に倒さず、通常動作を優先する。
        if self.latest_temperatures is None:
            return False
        return max(self.latest_temperatures) >= self.temp_threshold

    def _log(self, is_grasping):
        if self.log_file is None:
            return
        record = {
            'stamp': rospy.Time.now().to_sec(),
            'time': datetime.datetime.now().isoformat(),
            'is_grasping': is_grasping,
            'temperature_C': self.latest_temperatures,
            'current_mA': self.latest_currents,
        }
        try:
            self.log_file.write(json.dumps(record) + '\n')
            self.log_file.flush()
        except Exception as e:
            rospy.logwarn_throttle(10, 'ログ書き込みエラー: {}'.format(e))

    def _call_grasp(self, data):
        # data (true/false) を送る直前の温度・電流を記録する
        self._log(is_grasping=data)
        try:
            res = self.grasp(data=data)
            rospy.loginfo('data=%s -> success=%s, message="%s"', data, res.success, res.message)
        except rospy.ServiceException as e:
            rospy.logwarn('service call failed: %s', e)

    def run(self):
        data = False
        rate = rospy.Rate(1.0 / self.interval)
        while not rospy.is_shutdown():
            if self.is_overheated():
                if data:
                    rospy.logwarn_throttle(
                        5,
                        '過熱検知(%.1f度以上) - 握り動作を停止し、開いた状態を維持します',
                        self.temp_threshold,
                    )
                    self._call_grasp(False)
                    data = False
                else:
                    self._log(is_grasping=False)
                rate.sleep()
                continue

            self._call_grasp(data)
            data = not data
            rate.sleep()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--interval', type=float, default=5.0,
                         help='送信間隔 [sec] (default: 5.0)')
    parser.add_argument('--temp-threshold', type=float, default=70.0,
                         help='この温度[度C]以上のサーボが1つでもあれば握り動作を停止する (default: 70.0)')
    parser.add_argument('--log-dir', default='~/.ros/aero_hand_logs',
                         help='記録先ディレクトリ (default: ~/.ros/aero_hand_logs)')
    parser.add_argument('--no-log', action='store_true', help='温度・電流の記録を無効化する')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('toggle_grasp_service', anonymous=True)

    toggler = ThermalAwareGraspToggler(
        interval=args.interval,
        temp_threshold=args.temp_threshold,
        log_dir=args.log_dir,
        log_enable=not args.no_log,
    )
    toggler.run()


if __name__ == '__main__':
    main()
