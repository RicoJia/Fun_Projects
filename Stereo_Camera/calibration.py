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
SQUARE_SIDE=0.25    #in meters
IMAGE_NUM = 10
MARGIN_OFFSET = (1.2, 3.2)  #in checker squares
CACHE_DIR="rico_cache/"

class Calibrator(object):
    def __init__(self):
        # Teling the cv2 that we want to stop once max_iter, or convergence metrics reaches some small value. 
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # store vectors of 2D points for each checkerboard image
        self.imgpoints = np.array([])
        self.objp_all = []
        self.imgp_all = []
        self.params={}
        self.valid_img_num = 0
        self.__generate_objp()
        print("======================") 
        usage_msg = "[USAGE]: \nIf a chessboard is detected and we hit enter and a valid image is saved for calibration. \nWe need at least {IMAGE_NUM} valid images. \nTo save the parameters, hit y; to do another calibration session, hit n. "
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
        print("Convention: x-axis has more squares than y-axis")
        print(usage_msg) 
        print("======================") 
        logging.info("Calibrator Started") 

    def __generate_objp(self):
        # Defining the world coordinates for 3D points. NOTE: z is 0 because we assume z axis is pointing out of the plane
        # we populate the coordinate along x axis from origin outwards
        self.objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        self.objp[0,:,:2] = SQUARE_SIDE * (np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) + MARGIN_OFFSET)

    def detect_and_draw_chessboard_on_frame(self, frame) -> bool: 
        """
        If a chessboard of CHECKERBOARD is detected on the reference of a frame, then it will be drawn on the frame 
        return 1 if chessboard is detected. Else 0.
        """
        self.gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(self.gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret == True: 
            corners2 = cv2.cornerSubPix(self.gray, corners, (11,11),(-1,-1), self.criteria)
            frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)
            # refining pixel coordinates for given 2d points.
            self.__get_img_pts(corners2)
            #TODO
            self.frame = frame
        return ret

    def calibrate(self, key) -> bool: 
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
                success = self.__calibrate()
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


    def __calibrate(self) -> bool: 
        """
        Note: we assume neither +x or +y points to the ground direction. (-v value)
        return success - if more images are needed, will return false. Else return true
        """
        self.objp_all.append(self.objp)
        self.imgp_all.append(self.imgpoints)

        # #TODO: add offset
        # print("objp: ")
        # print(objp)
        # print("imgpoints: ")
        # print(self.imgpoints)

        # #TODO
        # new_frame = frame = cv2.putText(self.frame, f"(f{objp[0]})", (self.imgpoints[0, 0, 0], self.imgpoints[0, 0, 1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        # cv2.imshow("preview", new_frame)

        #TODO: to move 
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objp_all, self.imgp_all, self.gray.shape[::-1], None, None)
        self.imgpoints = np.array([])
        self.params["ret"] = ret
        self.params["tvecs"]=tvecs
        self.params["rvecs"]= rvecs
        self.params["dist"]= dist
        self.params["mtx"]= mtx

        logging.info("Successfully calibrated a camera")
        print("Camera matrix :")
        print(mtx)
        print("dist : ")
        print(dist)
        print("rvecs : ")
        print(rvecs)
        print("tvecs : ")
        print(tvecs)
        if self.valid_img_num == IMAGE_NUM: 
            self.__save_params_to_file(self.params)
            return True
        else: 
            logging.info(f"{IMAGE_NUM - self.valid_img_num} valid images are needed")
            return False

    def __save_params_to_file(self, params): 
        tm = calendar.timegm(time.gmtime()) 
        file_name = f"calibrate_{tm}.prm"
        with open(CACHE_DIR + file_name, "wb") as dfile:
            pickle.dump(params, dfile)

    def load_params_from_pickle(self):
        ls = listdir(CACHE_DIR)
        print(ls)
        if not listdir: 
            logging.warning(f"no calibration params are found in {CACHE_DIR}")
        else: 
            target_file_name = os.path.join(CACHE_DIR, max(ls))
            with open(target_file_name, "rb") as target_file: 
                self.params = pickle.load(target_file)
                print(self.params)

    def check_result(self, key):
        """
        Use the latest frame with chessboard and the saved intrinsics 
        Compute the extrinsics, and reproject obj points back onto the frame. So do not move the camera, you should see the same points projected back to the same positions
        """
        def show_frame(): 
            cv2.imshow("calibration", self.frame)
            key = cv2.waitKey(20)
            return key == ESC

        if key == ESC: 
            logging.info("Esc detected, no result checking is conducted")
            return True
        elif key ==ENTER and self.params and self.imgpoints.size!= 0: 
            success, rotation_vector, translation_vector = cv2.solvePnP(self.objp, self.imgpoints, self.params["mtx"], self.params["dist"], flags=0)
            if success: 
                reprojected_img_pts, jacobian = cv2.projectPoints(self.objp, rotation_vector, translation_vector, self.params["mtx"], self.params["dist"])
                print(reprojected_img_pts)
                for pt in reprojected_img_pts: 
                    cv2.circle(self.frame, (int(pt[0, 0]), int(pt[0, 1])), 3, (0, 255, 255), -1)
                while True: 
                    show_frame()
                    time.sleep(0.1)

            else: 
                logging.info("Pnp not solved sucessfully")
            logging.info("Result checked")
            return True
        else: 
            return False



        
