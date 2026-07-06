#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import SetBool, SetBoolResponse

class AeroHandGraspServiceNode:
    def __init__(self):
        rospy.init_node('aero_hand_grasp_service')

        # Load configuration parameters (defaults can be overridden via ROS parameter server)
        # Grasp targets
        self.grasp_abduction = rospy.get_param('~grasp_abduction', 0.0)
        self.grasp_thumb_add = rospy.get_param('~grasp_thumb_add', 0.7)
        self.grasp_thumb_flex = rospy.get_param('~grasp_thumb_flex', 0.6)
        self.grasp_index_flex = rospy.get_param('~grasp_index_flex', 0.6)
        self.grasp_middle_flex = rospy.get_param('~grasp_middle_flex', 0.6)
        self.grasp_ring_flex = rospy.get_param('~grasp_ring_flex', 0.6)
        self.grasp_little_flex = rospy.get_param('~grasp_little_flex', 0.6)

        # Open targets
        self.open_abduction = rospy.get_param('~open_abduction', 0.0)
        self.open_thumb_add = rospy.get_param('~open_thumb_add', 0.2)
        self.open_thumb_flex = rospy.get_param('~open_thumb_flex', 0.0)
        self.open_index_flex = rospy.get_param('~open_index_flex', 0.0)
        self.open_middle_flex = rospy.get_param('~open_middle_flex', 0.0)
        self.open_ring_flex = rospy.get_param('~open_ring_flex', 0.0)
        self.open_little_flex = rospy.get_param('~open_little_flex', 0.0)

        # Durations (seconds)
        self.thumb_add_duration = rospy.get_param('~thumb_add_duration', 1.0)
        self.fingers_duration = rospy.get_param('~fingers_duration', 1.0)
        self.open_duration = rospy.get_param('~open_duration', 1.0)

        # Thread safety lock to serialize service calls
        self.lock = threading.Lock()

        # Publisher to the hand controller command topic
        self.pub = rospy.Publisher('/aero_hand/command', JointTrajectory, queue_size=10)

        # Advertise the service
        self.srv = rospy.Service('~grasp', SetBool, self.handle_grasp)

        rospy.loginfo("AeroHand Grasp Service Node is ready.")
        rospy.loginfo("Service advertised: ~grasp")

    def send_hand_command(self, positions, duration_sec):
        """Helper function to construct and publish the JointTrajectory message"""
        msg = JointTrajectory()
        msg.joint_names = [
            "abduction",
            "thumb_add",
            "thumb_flex",
            "index_flex",
            "middle_flex",
            "ring_flex",
            "little_flex"
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration_sec)

        msg.points.append(point)
        self.pub.publish(msg)

    def handle_grasp(self, req):
        """Service callback to handle grasp (True) or open (False) requests"""
        with self.lock:
            if req.data:
                rospy.loginfo("Grasping sequence initiated...")
                
                # Step 1: Move thumb_add first (keep other joints at open position)
                rospy.loginfo(f"Step 1: Moving thumb_add to {self.grasp_thumb_add} (duration: {self.thumb_add_duration}s)...")
                step1_pose = [
                    self.grasp_abduction,
                    self.grasp_thumb_add,
                    self.open_thumb_flex,
                    self.open_index_flex,
                    self.open_middle_flex,
                    self.open_ring_flex,
                    self.open_little_flex
                ]
                self.send_hand_command(step1_pose, self.thumb_add_duration)
                rospy.sleep(self.thumb_add_duration)

                # Step 2: Flex thumb_flex and other 4 fingers simultaneously
                rospy.loginfo(f"Step 2: Flexing all fingers simultaneously (duration: {self.fingers_duration}s)...")
                step2_pose = [
                    self.grasp_abduction,
                    self.grasp_thumb_add,
                    self.grasp_thumb_flex,
                    self.grasp_index_flex,
                    self.grasp_middle_flex,
                    self.grasp_ring_flex,
                    self.grasp_little_flex
                ]
                self.send_hand_command(step2_pose, self.fingers_duration)
                rospy.sleep(self.fingers_duration)

                rospy.loginfo("Grasping sequence completed.")
                return SetBoolResponse(success=True, message="Grasped the human hand successfully.")

            else:
                rospy.loginfo("Opening sequence initiated...")
                
                # Move all joints back to open position simultaneously
                rospy.loginfo(f"Opening hand (duration: {self.open_duration}s)...")
                open_pose = [
                    self.open_abduction,
                    self.open_thumb_add,
                    self.open_thumb_flex,
                    self.open_index_flex,
                    self.open_middle_flex,
                    self.open_ring_flex,
                    self.open_little_flex
                ]
                self.send_hand_command(open_pose, self.open_duration)
                rospy.sleep(self.open_duration)

                rospy.loginfo("Opening sequence completed.")
                return SetBoolResponse(success=True, message="Opened the hand successfully.")

if __name__ == '__main__':
    try:
        node = AeroHandGraspServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
