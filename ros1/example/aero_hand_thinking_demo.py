#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# aeroの初期姿勢
# array([-0.8       ,  0.        ,  0.        , -0.        ,  0.        ,
#        -0.        , -3.        , -0.        ,  0.        , -0.        ,
#         0.        ,  0.8       , -0.        , -0.6       , -0.2       ,
#         0.2       , -2.5       , -0.7       ,  0.        ,  0.3       ,
#        -0.        , -0.        , -1.134464  ,  0.        ,  0.        ,
#         0.2617994 ,  0.87266463,  0.        ,  0.        , -0.87266463,
#         1.        , -1.        ,  1.        , -1.        ,  1.        ,
#         1.        ], dtype=float32)

def send_hand_command(pub, positions, duration_sec):
    """ JointTrajectoryメッセージを作成してパブリッシュする補助関数 """
    msg = JointTrajectory()
    msg.joint_names = ["abduction", "thumb_add", "thumb_flex", "index_flex", "middle_flex", "ring_flex", "little_flex"]

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(duration_sec)

    msg.points.append(point)
    pub.publish(msg)

def main():
    rospy.init_node('aero_hand_thinking_demo')

    pub = rospy.Publisher('/aero_hand/command', JointTrajectory, queue_size=10)

    rospy.loginfo("AeroHand 手招きデモを起動しました。開始まで3秒待ちます...")
    rospy.sleep(3.0)

    flex_duration = 0.7
    # ----------------------------

    # 初期位置（すべての指をリセットして伸ばす）
    open_pose = [0.0, 0.25, 0.15, 0.0, 0.0, 0.4, 0.4]
    rospy.loginfo("初期位置（全開）に移動します。")
    send_hand_command(pub, open_pose, 2.0)
    rospy.sleep(2)

    rospy.loginfo("ループを開始します。終了するには Ctrl+C を押してください。")

    while not rospy.is_shutdown():
        for i in range(5):
            send_hand_command(pub, [0.0, 0.25, 0.15, 0.3, 0.3, 0.4, 0.4], flex_duration)
            rospy.sleep(flex_duration)

            send_hand_command(pub, [0.0, 0.22, 0.12, 0.0, 0.0, 0.4, 0.4], flex_duration)
            rospy.sleep(flex_duration)

        rospy.sleep(3)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
