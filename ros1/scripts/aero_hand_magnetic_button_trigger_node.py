#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import math
from std_msgs.msg import Bool, String, Int32
from std_srvs.srv import SetBool, SetBoolRequest

def calculate_angle(v1, v2):
    """Calculate angle between two 3D vectors in degrees."""
    dot_product = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
    mag1 = math.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)
    mag2 = math.sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2)
    if mag1 == 0 or mag2 == 0:
        return 0.0
    cos_angle = dot_product / (mag1 * mag2)
    cos_angle = max(min(cos_angle, 1.0), -1.0)
    return math.degrees(math.acos(cos_angle))

class MagneticButtonTriggerNode:
    def __init__(self):
        rospy.init_node('aero_hand_magnetic_button_trigger')

        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.angle_threshold = rospy.get_param('~angle_threshold', 15.0) # degrees
        self.force_release_time = rospy.get_param('~force_release_time', 20.0) # seconds
        self.target_mode = rospy.get_param('~target_mode', 'DisplayInformationMode')

        self.state = 'WAITING' # States: 'WAITING', 'GRASPED', 'COOLDOWN'
        self.grasp_time = None
        self.is_service_running = False

        rospy.loginfo("Waiting for grasp service (/aero_hand_grasp_service/grasp)...")
        rospy.wait_for_service('/aero_hand_grasp_service/grasp')
        self.grasp_service = rospy.ServiceProxy('/aero_hand_grasp_service/grasp', SetBool)

        self.grasp_pub = rospy.Publisher('/aero_hand/is_grasped', Bool, queue_size=1, latch=True)
        self.grasp_pub.publish(Bool(data=False))

        self.initial_vector = None
        self.calibration_samples = []

        self.current_mode = None
        self.last_button_state = None
        self.active = False
        self.read_thread = None
        self.toggle_lock = threading.Lock()

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            rospy.loginfo(f"Connected to {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            return

        rospy.Subscriber('/atom_s3_mode', String, self.mode_callback, queue_size=1)
        rospy.Subscriber('/atom_s3_button_state', Int32, self.button_callback, queue_size=10)

        rospy.loginfo(f"Magnetic button trigger node started. Press the button in '{self.target_mode}' to start/stop.")
        rospy.loginfo(f"Parameters -> threshold: {self.angle_threshold} deg, force release: {self.force_release_time} s.")

    def call_service(self, req):
        try:
            self.is_service_running = True
            self.grasp_service(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        finally:
            self.is_service_running = False


    def mode_callback(self, msg):
        self.current_mode = msg.data

    def button_callback(self, msg):
        pressed = (msg.data == 1 and self.last_button_state != 1)
        self.last_button_state = msg.data
        if not pressed or self.current_mode != self.target_mode:
            return
        with self.toggle_lock:
            if self.active:
                self.stop_feedback()
            else:
                self.start_feedback()

    def start_feedback(self):
        # Wait for the previous loop to exit completely
        if self.read_thread is not None and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)

        # Reset calibration and state so that calibration is performed every time
        self.initial_vector = None
        self.calibration_samples = []
        self.state = 'WAITING'
        self.grasp_time = None
        try:
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            rospy.logwarn(f"Failed to flush serial buffer: {e}")

        self.active = True
        self.read_thread = threading.Thread(target=self.serial_read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        rospy.loginfo("Button pressed. Sensor feedback started (calibrating...).")

    def stop_feedback(self):
        self.active = False
        if self.state == 'GRASPED':
            rospy.loginfo("Sensor feedback stopped while grasping. Releasing grasp (False)!")
            req = SetBoolRequest(data=False)
            threading.Thread(target=self.call_service, args=(req,)).start()
            self.grasp_pub.publish(Bool(data=False))
        self.state = 'WAITING'
        rospy.loginfo("Button pressed. Sensor feedback stopped.")

    def serial_read_loop(self):
        while not rospy.is_shutdown() and self.active:
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
                        # Average first 50 samples for initial reference
                        if len(self.calibration_samples) < 50:
                            continue
                        
                        avg_x = sum(v[0] for v in self.calibration_samples) / 50.0
                        avg_y = sum(v[1] for v in self.calibration_samples) / 50.0
                        avg_z = sum(v[2] for v in self.calibration_samples) / 50.0
                        self.initial_vector = (avg_x, avg_y, avg_z)
                        rospy.loginfo(f"Calibration complete. Initial vector: {self.initial_vector}")
                        continue

                    if not self.active:
                        break

                    angle = calculate_angle(self.initial_vector, current_vector)
                    rospy.loginfo(f"Current angle: {angle:.2f} deg")
                    
                    if self.state == 'WAITING':
                        if angle > self.angle_threshold and not self.is_service_running:
                            rospy.loginfo(f"Tilt angle ({angle:.2f} deg) > {self.angle_threshold}. Triggering grasp (True)!")
                            self.state = 'GRASPED'
                            self.grasp_time = rospy.Time.now()
                            req = SetBoolRequest(data=True)
                            threading.Thread(target=self.call_service, args=(req,)).start()
                            self.grasp_pub.publish(Bool(data=True))
                            
                    elif self.state == 'GRASPED':
                        # Wait until the sensor returns to near its original tilt
                        if angle < self.angle_threshold * 0.4 and not self.is_service_running:
                            rospy.loginfo(f"Tilt returned to normal ({angle:.2f} deg). Releasing grasp (False)!")
                            self.state = 'WAITING'
                            req = SetBoolRequest(data=False)
                            threading.Thread(target=self.call_service, args=(req,)).start()
                            self.grasp_pub.publish(Bool(data=False))
                        # Or wait until 20 seconds have elapsed to force release
                        elif (rospy.Time.now() - self.grasp_time).to_sec() > self.force_release_time and not self.is_service_running:
                            rospy.loginfo(f"{self.force_release_time} seconds elapsed. Force releasing grasp (False)!")
                            self.state = 'COOLDOWN'
                            req = SetBoolRequest(data=False)
                            threading.Thread(target=self.call_service, args=(req,)).start()
                            self.grasp_pub.publish(Bool(data=False))
                            
                    elif self.state == 'COOLDOWN':
                        # Wait until the sensor returns to near its original tilt to reset
                        if angle < self.angle_threshold * 0.4:
                            rospy.loginfo(f"Tilt returned to normal ({angle:.2f} deg). Ready for next trigger.")
                            self.state = 'WAITING'

            except ValueError:
                pass # Ignore parsing errors for partial lines
            except serial.SerialException as e:
                rospy.logerr(f"Serial read error: {e}")
                rospy.sleep(1.0)
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")

if __name__ == '__main__':
    try:
        node = MagneticButtonTriggerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
