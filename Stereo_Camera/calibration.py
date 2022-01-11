import logging 
import cv2
import numpy as np
import os
import glob

import calendar
import time
import pickle
from os import listdir

import time

CHECKERBOARD = (7, 4)   #Convention: (x, y)
ENTER = 13
ESC = 27
SQUARE_SIDE=0.016    # black/ white in meters
IMAGE_NUM = 10
MARGIN_OFFSET = (1.2, 3.2)  #in checker squares
PARAM_DIR="camera_calibration_params/"
LEFT=0
RIGHT=1

def print_dict(dic):
    """

    :dic: TODO
    :returns: TODO

    """
    for key, value in dic.items(): 
        print(key, ": ")
        print(value)

class Calibrator(object):
    def __init__(self, camera_name="", is_fish_eye=False):
        # Teling the cv2 that we want to stop once max_iter, or convergence metrics reaches some small value. 
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # store vectors of 2D points for each checkerboard image
        self.imgpoints = np.array([])
        self.objp_all = []
        self.imgp_all = []
        self.params={}
        self.gray_image_size = None
        self.valid_img_num = 0
        self.camera_name = camera_name
        self.is_fish_eye = is_fish_eye
        self.__generate_objp()
        print("======================") 
        usage_msg = "[USAGE]: \nIf a chessboard is detected and we hit enter and a valid image is saved for calibration. \nWe need at least {IMAGE_NUM} valid images. \nTo save the parameters, hit y; to do another calibration session, hit n. "
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
        print("Convention: x-axis has more squares than y-axis")
        print(usage_msg) 
        print("======================") 
        logging.info(f"Calibrator: {self.camera_name} Started") 

    def __generate_objp(self):
        # Defining the world coordinates for 3D points. NOTE: z is 0 because we assume z axis is pointing out of the plane
        # we populate the coordinate along x axis from origin outwards
        self.objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        self.objp[0,:,:2] = 2 * SQUARE_SIDE * (np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) + MARGIN_OFFSET)

    def detect_and_draw_chessboard_on_frame(self, frame) -> bool: 
        """
        If a chessboard of CHECKERBOARD is detected on the reference of a frame, then it will be drawn on the frame 
        return 1 if chessboard is detected. Else 0.
        """
        self.gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        self.gray_image_size = self.gray.shape
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(self.gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret == True: 
            corners2 = cv2.cornerSubPix(self.gray, corners, (11,11),(-1,-1), self.criteria)
            frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)
            # refining pixel coordinates for given 2d points.
            self.__get_img_pts(corners2)
            self.frame = frame
        return ret

    def calibrate(self, key, just_prepare_img_pts = False) -> bool: 
        """
        return success - 1 if we have successfully calibrated a camera. Else 0.
        """
        if key == ESC: 
            logging.info("Esc detected, no calibration is conducted")
            return True
        elif key ==ENTER:
            if self.imgpoints.size == 0: 
                logging.warning("No chessboard detected successfully. Wait for a chessboard to show up or enter Esc to quit")
                return False
            else: 
                # will use the most recent img
                self.valid_img_num += 1
                success = self.__calibrate(just_prepare_img_pts)
                return success
        else:
            return False

    def __get_img_pts(self, corners2):
        """
        """
        #compare v value in image coord (u,v), so we always add points along x axis. 
        self.imgpoints = corners2
        if self.imgpoints[0,0,1] < self.imgpoints[-1, 0, 1]: 
            self.imgpoints = np.flipud(self.imgpoints)


    def __calibrate(self, just_prepare_img_pts) -> bool: 
        """
        Note: we assume neither +x or +y points to the ground direction. (-v value)
        return success - if more images are needed, will return false. Else return true
        """
        self.objp_all.append(self.objp)
        self.imgp_all.append(self.imgpoints)

        if self.valid_img_num == IMAGE_NUM: 
            if not just_prepare_img_pts: 
                if self.is_fish_eye: 
                    mtx = np.zeros((3, 3))
                    dist = np.zeros((4, 1))
                    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(IMAGE_NUM)]
                    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(IMAGE_NUM)]
                    ret, mtx, dist, rvecs, tvecs = cv2.fisheye.calibrate(
                        self.objp_all,
                        self.imgp_all,
                        self.gray.shape,
                        mtx,
                        dist,
                        rvecs,
                        tvecs,
                        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_FIX_SKEW,
                        self.criteria
                    )
                else: 
                    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objp_all, self.imgp_all, self.gray.shape, None, None)

                self.imgpoints = np.array([])
                self.params["ret"] = ret
                self.params["tvecs"]=tvecs
                self.params["rvecs"]= rvecs
                self.params["dist"]= dist
                self.params["mtx"]= mtx

                logging.info("Successfully calibrated a camera")
                print("ret: ")
                print(ret)
                print("Camera matrix :")
                print(mtx)
                print("dist : ")
                print(dist)
                self.__save_params_to_file(self.params)
            self.valid_img_num = 0
            return True
        else: 
            logging.info(f"{IMAGE_NUM - self.valid_img_num} valid images are needed")
            return False

    def __save_params_to_file(self, params): 
        tm = calendar.timegm(time.gmtime()) 
        file_name = f"{self.camera_name}_calibrate_{tm}.prm"
        with open(PARAM_DIR + file_name, "wb") as dfile:
            pickle.dump(params, dfile)

#========================Run Time #========================
    def load_params_from_pickle(self):
        """
        Load the last written file in pickle
        """
        ls = listdir(PARAM_DIR)
        ls = list(filter(lambda x: self.camera_name in x, ls))
        if not ls: 
            logging.warning(f"no calibration params are found under {self.camera_name} in {PARAM_DIR}")
        else: 
            target_file_name = os.path.join(PARAM_DIR, max(ls))
            with open(target_file_name, "rb") as target_file: 
                self.params = pickle.load(target_file)
                print_dict(self.params)
            logging.info(f"{self.camera_name}: loaded params sucessfully")

    def undistort_frame(self, frame): 
        """
        return frame
        """
        self.gray_image_size = frame.shape[:2]
        new_camera_mtx=np.array([])
        if self.is_fish_eye: 
            new_camera_mtx = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.params["mtx"], self.params["dist"], self.gray_image_size, None)
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(self.params["mtx"], self.params["dist"], np.eye(3), new_camera_mtx, self.gray_image_size, cv2.CV_16SC2)
            return cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        else: 
            return cv2.undistort(frame, self.params["mtx"], self.params["dist"], None, new_camera_mtx)

    def check_result(self, key) -> bool:
        """
        Use the latest frame with chessboard and the saved intrinsics 
        Compute the extrinsics, and reproject obj points back onto the frame. So do not move the camera, you should see the same points projected back to the same positions
        return done_checking
        """
        def show_frame(): 
            cv2.imshow(f"Result Checking - {self.camera_name}", self.frame)
            key = cv2.waitKey(20)
            return key != ENTER

        if key == ESC: 
            logging.info("f{self.camera_name}: Esc detected, no result checking is conducted")
            return True
        elif key ==ENTER and self.params and self.imgpoints.size!= 0: 
            success, rotation_vector, translation_vector = cv2.solvePnP(self.objp, self.imgpoints, self.params["mtx"], self.params["dist"], flags=0)
            if success: 
                reprojected_img_pts, jacobian = cv2.projectPoints(self.objp, rotation_vector, translation_vector, self.params["mtx"], self.params["dist"])
                for pt in reprojected_img_pts: 
                    cv2.circle(self.frame, (int(pt[0, 0]), int(pt[0, 1])), 3, (0, 255, 255), -1)
                # undistort the image
                self.frame = self.undistort_frame(self.frame)
                logging.info(f"{self.camera_name}: showing check result - press ENTER to unfreeze")
                while show_frame():
                    time.sleep(0.01)
            else: 
                logging.info("Pnp not solved sucessfully")
            return False
        else: 
            return False

    # ====================== For Stereo Calibration ======================
    def prepare_data_for_stereo_calibration(self, key): 
        """
        Use this function AFTER sucessfully calibrated an individual camera, and taking in ALL images for calibration. 
        We are using the same facility in calibrate
        """
        return self.calibrate(key, just_prepare_img_pts=True)

    def get_data_for_stereo_calibration(self):
        return {"all_object_points": self.objp_all, "all_img_points": self.imgp_all, "mtx": self.params["mtx"], "dist": self.params["dist"], "img_size": self.gray.shape}


class StereoCalibrator(object): 
    """
    Object that takes in: 
        1. extrinsics and intrinsics from left and right cameras
        2. stores stereo information. 
    During runtime, it will: 
        1. Take in undistorted images from two cameras
        2. Rectify them.
    """
    def __init__(self, window_name, left_camera_params, right_camera_params):
        self.window_name = window_name
        self.single_camera_params_ls = []
        self.single_camera_params_ls = [left_camera_params, right_camera_params]

    # ====================== Calibration ======================
    def load_params(self): 
        """
        Load single camera params and stereo camera params
        """
        ls = listdir(PARAM_DIR)
        ls = list(filter(lambda x: self.window_name in x, ls))
        if not ls: 
            logging.warning(f"no calibration params are found under {self.window_name} in {PARAM_DIR}")
        else: 
            target_file_name = os.path.join(PARAM_DIR, max(ls))
            with open(target_file_name, "rb") as target_file: 
                self.stereo_camera_params = pickle.load(target_file)
                print_dict(self.stereo_camera_params)
            logging.info(f"{self.window_name}: loaded params sucessfully")

    def __save_params_to_file(self, params, name_header): 
        tm = calendar.timegm(time.gmtime()) 
        file_name = f"{name_header}_calibrate_{tm}.prm"
        with open(PARAM_DIR + file_name, "wb") as dfile:
            pickle.dump(params, dfile)
    
    def calibrate(self,  left_camera_data, right_camera_data):
        """
        Stereo Calibration contains: 
            1. finding out R, T (SE(3) Transform cam1 -> cam2)
            2. 
        left_camera_data, right_camera_data are dict with these fields
        {["all_object_points"]: , ["all_img_points"]: , ["mtx"]:, ["dist"]: , ["img_size"]}
        """
        stereocalibration_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 0.001)
        # stereocalibration_flags = cv2.CALIB_FIX_INTRINSIC
        stereocalibration_flags = cv2.CALIB_USE_INTRINSIC_GUESS
        ret, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
                left_camera_data["all_object_points"], 
                left_camera_data["all_img_points"], right_camera_data["all_img_points"], 
                left_camera_data["mtx"], left_camera_data["dist"], 
                right_camera_data["mtx"], right_camera_data["dist"], 
                left_camera_data["img_size"], 
                stereocalibration_criteria, 
                stereocalibration_flags
                )
        self.stereo_camera_params = {} 
        self.stereo_camera_params["ret"] = ret
        self.stereo_camera_params["R"] = R
        self.stereo_camera_params["T"] = T
        self.stereo_camera_params["E"] = E
        self.stereo_camera_params["F"] = F
        self.stereo_camera_params["img_size"] = left_camera_data["img_size"]

        #TODO
        print("stereo params: ")
        print_dict(self.stereo_camera_params)

