#! /usr/bin/python3
'''
This script is to: 
    1. Listen to tf: base_link -> if 
    2. Detect Aruco Marker pose(s), relative to the depth camera
    3. Apply hand-eye-calibration by solving AX = XB, by using OpenCV: 
        https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b
'''
from aruco_detection import ArucoDetector
import rospy
import cv2
import tf2_ros
import tf_conversions.posemath
class HandEyeCalib3d:
    def __init__(self) -> None:
        self.aruco_detector = ArucoDetector(visualize=True)
        self.tf_buffer = tf2_ros.Buffer(rospy.Time(1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def calibrate(self):
        try:
            T_base_link_2_link_6 = self.tf_buffer.lookup_transform("base_link", "link_6", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            raise "There's an error in calibration: "+ e
        #TODO
        print("trans: ", T_base_link_2_link_6)
        # TODO: what is rvec? Quaternion, or Euler Angles?
        # gTb =  T_gripper_base
        # bTc = T_base_camera
        # cTt = T_camera_board
        # X = X^(-1)
        # T_base_cam = cv2.calibrateHandEye(rvec_gripper_base, tvec_gripper_base, rvec_board_cam, tvec_board_cam)
        
         
if __name__ == "__main__":
    rospy.init_node("hand_eye_calib")     
    rate = rospy.Rate(30)
    hec = HandEyeCalib3d()
    input("Press any key to start calibration")
    hec.calibrate()