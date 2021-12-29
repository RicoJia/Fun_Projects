import logging 
import cv2
import numpy as np
import os
import glob

CHECKERBOARD = (7, 4)   #Convention: (x, y)
ENTER = 13
ESC = 27
SQUARE_SIDE=0.25    #in meters
IMAGE_NUM = 20
MARGIN_OFFSET = (1.2, 3.2)  #in checker squares

class Calibrator(object):
    def __init__(self):
        # Teling the cv2 that we want to stop once max_iter, or convergence metrics reaches some small value. 
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # store vectors of 2D points for each checkerboard image
        self.imgpoints = np.array([])
        self.objp_all = []
        self.imgp_all = []
        self.valid_img_num = 0
        print("======================") 
        usage_msg = "[USAGE]: \nIf a chessboard is detected and we hit enter and a valid image is saved for calibration. \nWe need at least {IMAGE_NUM} valid images. \nTo save the parameters, hit y; to do another calibration session, hit n. "
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
        print("Convention: x-axis has more squares than y-axis")
        print(usage_msg) 
        print("======================") 
        logging.info("Calibrator Started") 

    def detect_and_draw_chessboard_on_frame(self, frame): 
        """
        If a chessboard of CHECKERBOARD is detected on the reference of a frame, then it will be drawn on the frame 
        """
        self.gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(self.gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret == True: 
            corners2 = cv2.cornerSubPix(self.gray, corners, (11,11),(-1,-1), self.criteria)
            frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)
            # refining pixel coordinates for given 2d points.
            self.imgpoints = corners2
            #TODO
            self.frame = frame

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
                self.imgpoints = np.array([])
                return success
        else:
            return False

    def __calibrate(self) -> bool: 
        """
        Note: we assume neither +x or +y points to the ground direction. (-v value)
        return success - if more images are needed, will return false. Else return true
        """
        #compare v value in image coord (u,v), so we always add points along x axis. 
        if self.imgpoints[0,0,1] < self.imgpoints[-1, 0, 1]: 
            self.imgpoints = np.flipud(self.imgpoints)
        # Defining the world coordinates for 3D points. NOTE: z is 0 because we assume z axis is pointing out of the plane
        # we populate the coordinate along x axis from origin outwards
        objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[0,:,:2] = SQUARE_SIDE * (np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) + MARGIN_OFFSET)
        self.objp_all.append(objp)
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
            return True
        else: 
            logging.info(f"{IMAGE_NUM - self.valid_img_num} valid images are needed")
            return False
