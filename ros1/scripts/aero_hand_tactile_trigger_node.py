#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
from std_msgs.msg import Int32MultiArray, Bool
from std_srvs.srv import SetBool, SetBoolRequest

class TactileTriggerNode:
    def __init__(self):
        rospy.init_node('aero_hand_tactile_trigger')

        self.threshold = 40
        self.is_grasped = False
        self.is_service_running = False

        rospy.loginfo("Waiting for grasp service (/aero_hand_grasp_service/grasp)...")
        rospy.wait_for_service('/aero_hand_grasp_service/grasp')
        self.grasp_service = rospy.ServiceProxy('/aero_hand_grasp_service/grasp', SetBool)

        self.grasp_pub = rospy.Publisher('/aero_hand/is_grasped', Bool, queue_size=1, latch=True)
        # Publish initial state (False)
        self.grasp_pub.publish(Bool(data=False))

        self.sub = rospy.Subscriber('/aero_hand/tactile', Int32MultiArray, self.tactile_callback)
        rospy.loginfo(f"Tactile trigger node started. Threshold set to {self.threshold}.")

    def call_service_thread(self, req):
        try:
            self.grasp_service(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        finally:
            self.is_service_running = False

    def tactile_callback(self, msg):
        if self.is_service_running:
            return

        if not msg.data:
            return

        tactile_sum = sum(msg.data)
        
        if tactile_sum > self.threshold and not self.is_grasped:
            rospy.loginfo(f"Tactile sum ({tactile_sum}) > {self.threshold}. Triggering grasp (True)!")
            self.is_grasped = True
            self.is_service_running = True
            req = SetBoolRequest(data=True)
            threading.Thread(target=self.call_service_thread, args=(req,)).start()
            self.grasp_pub.publish(Bool(data=True))
                
        elif tactile_sum < self.threshold and self.is_grasped:
            rospy.loginfo(f"Tactile sum ({tactile_sum}) < {self.threshold}. Releasing grasp (False)!")
            self.is_grasped = False
            self.is_service_running = True
            req = SetBoolRequest(data=False)
            threading.Thread(target=self.call_service_thread, args=(req,)).start()
            self.grasp_pub.publish(Bool(data=False))

if __name__ == '__main__':
    try:
        node = TactileTriggerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
