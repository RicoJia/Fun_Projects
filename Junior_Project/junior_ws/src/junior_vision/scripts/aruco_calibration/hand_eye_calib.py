#! /usr/bin/env python3
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
from SimpleRoboticsPythonUtils import transform_to_cv2_rotation_tvec, ask_yes_no
MIN_VALID_SHOTS_REQUIRED = 3

pref = "\033["
reset = f"{pref}0m"
class Colors:
    black = "30m"
    red = "31m"
    green = "32m"
    yellow = "33m"
    blue = "34m"
    magenta = "35m"
    cyan = "36m"
    white = "37m"

# Alternative to print, uses white color by default but accepts any color 
# from the Colors class. Name it as you like.
def puts(text, color=Colors.black, is_bold=False):
    print(f'{pref}{1 if is_bold else 0};{color}' + text + reset)

def yes_no_question(question = str, color: str = Colors.black, is_bold=False):
    while True:
        puts(question, color=color, is_bold=is_bold)
        resp = input("[Yes y or [Enter] / No n/ Abort q]\n")
        if resp == "q":
            exit(1)
        elif resp.lower() in ("", "y"):
            return True
        elif resp.lower() == "n":
            return False
    

import os
class DirPath:
    '''
    Descriptor class with checking for & representing directory
    '''
    def __init__(self, dir_path: str):
        self.dir_path = dir_path

    def __get__(self, instance, owner_cls):
        if instance is not None:
            return instance.__dict__[self.dir_path]
        else: 
            return self

    def __set__(self, instance, value):
        value = os.path.expanduser(value)
        if not os.path.isdir(value):
            create_dir = yes_no_question(f"Dir {value} does not exist. Would you like to create it?") 
            if create_dir:
                os.makedirs(value)
                puts(f"Directory {value} is created")
            else:
                puts(f"Directory {value} is not created. ") 
        self.dir_path = value
        instance.__dict__[self.dir_path] = self.dir_path
        
import time, datetime
def strf_timestamp_utc():
    timestamp = time.time()
    date_time = datetime.datetime.fromtimestamp(timestamp)
    strf_date_time = date_time.strftime("%d-%m-%Y, %H:%M:%S") 
    return strf_date_time

from dataclasses import dataclass
import pickle
import logging
@dataclass
class DataSaveLoad:
    file_path: DirPath = DirPath("")
    file_name: str= ""
    suffix: str = "" 

    def save(self, data_dict: dict = {}, time_stamped: bool = False):
        if not data_dict:
            return

        if not isinstance(data_dict, dict):
            raise TypeError("Must pass a dict {'data_source_name': data ... } into save function")
        if time_stamped:
            file_name = self.file_name + strf_timestamp_utc() + "." + self.suffix
        else: 
            file_name = self.file_name + "." + self.suffix
        full_file_path = os.path.join(self.file_path, file_name)
        #TODO
        print(self.file_path)
        with open(full_file_path, "wb") as f:
            pickle.dump(data_dict, f)
            puts("Data successfully saved", Colors.green, is_bold=True) 

    def load(self):
        """
        Load the last written file with the desired file_name in file_path
        """
        ls = os.listdir(self.file_path)
        ls = list(filter(lambda x: self.file_name in x, ls))
        if not ls: 
            logging.warning(f"no calibration params are found under {self.camera_name} in {PARAM_DIR}")
            return {}
        else: 
            target_file_name = os.path.join(self.file_path, max(ls))
            with open(target_file_name, "rb") as target_file: 
                data_dict = pickle.load(target_file)
            return data_dict
            

class HandEyeCalib3d:
    def __init__(self, record_data = False) -> None:
        # We are not using the default visualization in ArucoDetector
        # Instead, turn on visualization with key presses
        self.aruco_detector = ArucoDetector(visualize=False, marker_size=0.036)
        self.tf_buffer = tf2_ros.Buffer(rospy.Time(1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.record_data = record_data

        print("=================")
        print("Welcome to Hand Eye Calibration! 🔭🚀")

    def __collect_calibration_data(self):
        puts("Data Collection Started!", is_bold=True)
        puts(f"Make sure there are at least {MIN_VALID_SHOTS_REQUIRED} shots taken!", Colors.magenta)
        puts("Hit y to take a shot, q to quit", is_bold=True)
        imgs = []
        tfs = []
        while True:
            color_img, corners, ids = self.aruco_detector.detect_aruco()
            original_color_img = color_img.copy()
            _, rvecs, tvecs = self.aruco_detector.estimate_aruco_poses(color_img, corners, ids)
            key_pressed = self.aruco_detector.show_aruco(color_img, corners, ids, rvecs, tvecs)
            if key_pressed == ord("q"):
                puts("Calibration Data Collection Done", Colors.cyan, is_bold=True)
                break
            elif key_pressed == ord("y"): 
                if ids is None:
                    puts("Please make sure there is at least one Aruco Marker in view", Colors.red)
                elif len(ids) > 1:
                    puts("Please make sure there's only one AruCo marker in camera view", Colors.yellow)
                else:
                    puts("Saved shot", Colors.green) 
                    imgs.append(original_color_img)
                try:
                    T_base_link_2_link_6 = self.tf_buffer.lookup_transform("base_link", "link_6", rospy.Time())
                    tfs.append(T_base_link_2_link_6)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    raise ValueError(e) from None

        return imgs, tfs
        
    def calibrate(self):
        imgs, tfs = self.__collect_calibration_data()
        dsl = DataSaveLoad("~/tmp_data_not_exist", file_name = "hand_eye_calib", suffix = "data")
        if self.record_data:
            puts("Saving Data ... ", Colors.blue, is_bold=True)
            dsl.save({"imgs": imgs}, time_stamped=True)
        data = dsl.load() 

        # T_base_link_2_link_6s = []
        # rvec_gripper_base_ls = []
        # tvec_gripper_base_ls = []
        rvec_board_cam_ls = []
        tvec_board_cam_ls = []
        logging.info("Calibration Started...")
        for color_img in data["imgs"]:
            # Now we are going to calibrate
            color_img, corners, ids = self.aruco_detector.detect_aruco()
            _, rvecs, tvecs = self.aruco_detector.estimate_aruco_poses(color_img, corners, ids)
            self.aruco_detector.show_aruco(color_img, corners, ids, rvecs, tvecs, show_static_img=True)
            # T_base_link_2_link_6s.append(T_base_link_2_link_6)
            rvec_board_cam_ls.append(rvecs[0][0])
            tvec_board_cam_ls.append(tvecs[0][0])

        #             T, t = transform_to_cv2_rotation_tvec(cv2, T_base_link_2_link_6.transform)
        #             rvec_gripper_base_ls.append(T) 
        #             tvec_gripper_base_ls.append(t) 
        #             rospy.logdebug("trans: ", T_base_link_2_link_6s)
        # TODO: what is rvec? theta * rotation axis
        # gTb =  T_gripper_base
        # bTc = T_base_camera
        # cTt = T_camera_board
        # X = X^(-1)
        # if len(T_base_link_2_link_6s) > 3:
        #     T_base_cam = cv2.calibrateHandEye(rvec_gripper_base_ls, tvec_gripper_base_ls, rvec_board_cam_ls, tvec_board_cam_ls)
        #     print(T_base_cam)
        
         
if __name__ == "__main__":
    rospy.init_node("hand_eye_calib")     
    rate = rospy.Rate(30)
    record_data = rospy.get_param("~record_data", False)
    hec = HandEyeCalib3d(record_data)
    hec.calibrate()