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
from SimpleRoboticsPythonUtils import transform_to_cv2_rvec_tvec, ask_yes_no
MIN_VALID_SHOTS_REQUIRED = 3
class HandEyeCalib3d:
    def __init__(self) -> None:
        # We are not using the default visualization in ArucoDetector
        # Instead, turn on visualization with key presses
        self.aruco_detector = ArucoDetector(visualize=False)
        self.tf_buffer = tf2_ros.Buffer(rospy.Time(1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        print("=================")
        print("Welcome to Hand Eye Calibration! 🔭🚀")
        print("Press q to stop data collection")
        print("Press Y to Accept View for calibration")
        print(f"Make sure there are at least {MIN_VALID_SHOTS_REQUIRED} shots taken!")

    def calibrate(self):
        T_base_link_2_link_6s = []
        rvec_gripper_base_ls = []
        tvec_gripper_base_ls = []
        rvec_board_cam_ls = []
        tvec_board_cam_ls = []
        
        while True:
            color_img, corners, ids = self.aruco_detector.detect_aruco()
            _, rvecs, tvecs = self.aruco_detector.estimate_aruco_poses(color_img, corners, ids)
            key_pressed = self.aruco_detector.show_aruco(color_img, corners, ids, rvecs, tvecs)
            if key_pressed == ord("q"):
                print("Hand eye calib finished, bye!")
                break
            elif key_pressed == ord("y"): 
                if len(ids) == 0:
                    continue
                elif len(ids) > 1:
                    print("Please make sure there's only one AruCo marker in camera view")
                    continue
                # Now we are going to calibrate
                try:
                    T_base_link_2_link_6 = self.tf_buffer.lookup_transform("base_link", "link_6", rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    raise ValueError("Please check if robot is publishing TF!" + e) from None
                else:
                    T_base_link_2_link_6s.append(T_base_link_2_link_6)
                    rvec_gripper_base_ls.append(rvecs[0][0])
                    tvec_gripper_base_ls.append(tvecs[0][0])

                    transform_to_cv2_rvec_tvec(cv2, T_base_link_2_link_6.transform)
                    
                    print("Shot added")
                    rospy.logdebug("trans: ", T_base_link_2_link_6s)
                # # TODO: what is rvec? Quaternion, or Euler Angles?
                # # gTb =  T_gripper_base
                # # bTc = T_base_camera
                # # cTt = T_camera_board
                # # X = X^(-1)
        print("rvecs: ", rvec_gripper_base_ls, " tvecs: ",tvec_gripper_base_ls)
        # if len(T_base_link_2_link_6s) > 3):
            # T_base_cam = cv2.calibrateHandEye(rvec_gripper_base, tvec_gripper_base, rvec_board_cam, tvec_board_cam)
        
         
if __name__ == "__main__":
    rospy.init_node("hand_eye_calib")     
    rate = rospy.Rate(30)
    hec = HandEyeCalib3d()
    hec.calibrate()