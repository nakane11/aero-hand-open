#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_hand_command(pub, positions, duration_sec):
    """ JointTrajectoryメッセージを作成してパブリッシュする補助関数 """
    msg = JointTrajectory()
    # 念のため関節名を入れておきます（インターポレータ側では不使用）
    msg.joint_names = ["abduction", "thumb_add", "thumb_flex", "index_flex", "middle_flex", "ring_flex", "little_flex"]

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(duration_sec)

    msg.points.append(point)
    pub.publish(msg)

def main():
    rospy.init_node('aero_hand_beckoning_demo')

    # インターポレータのレシーバートピックへパブリッシュ
    pub = rospy.Publisher('/aero_hand/command', JointTrajectory, queue_size=10)

    rospy.loginfo("AeroHand 手招きデモを起動しました。開始まで3秒待ちます...")
    rospy.sleep(3.0)

    # --- パラメータ調整エリア ---
    # 0.0 = 全開, 1.0 = 全閉（手招きなので 0.85 くらいが自然に綺麗に見えます）
    flex_val = 0.5

    # 演出用のタイマ設定（秒）
    phase_delay = 0.06   # 指と指の間の時間差（生き物らしさを生むコアパラメータ）
    flex_duration = 0.3 # 各指が動き始めてから曲がりきるまでの時間
    hold_time = 0.05     # 握りきった状態をキープする時間
    open_duration = 0.3 # 指を一気に開く時間
    loop_delay = 0.05    # 次の手招き動作に移る前の余韻
    # ----------------------------

    # 初期位置（すべての指をリセットして伸ばす）
    open_pose = [0.0, 0.20, 0.0, 0.0, 0.0, 0.0, 0.0]
    rospy.loginfo("初期位置（全開）に移動します。")
    send_hand_command(pub, open_pose, 1.0)
    rospy.sleep(1.2)

    rospy.loginfo("手招きループを開始します。終了するには Ctrl+C を押してください。")

    while not rospy.is_shutdown():
        for i in range(3):
            send_hand_command(pub, [0.0, 0.15, 0.2, 0.0, 0.0, 0.0, flex_val], flex_duration)
            rospy.sleep(phase_delay)

            send_hand_command(pub, [0.0, 0.15, 0.2, 0.0, 0.0, flex_val, flex_val], flex_duration)
            rospy.sleep(phase_delay)

            send_hand_command(pub, [0.0, 0.15, 0.2, 0.0, flex_val, flex_val, flex_val], flex_duration)
            rospy.sleep(phase_delay)

            send_hand_command(pub, [0.0, 0.15, 0.2, flex_val, flex_val, flex_val, flex_val], flex_duration)

            rospy.sleep(flex_duration + hold_time)

            send_hand_command(pub, open_pose, open_duration)

            rospy.sleep(open_duration + loop_delay)
        rospy.sleep(3)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
