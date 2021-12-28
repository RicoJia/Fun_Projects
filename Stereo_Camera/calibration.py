# Procedure 
# Place the 3D checker board along the positive directions of x,y,z
# get gray scale 
# detect the keyboard, locate the grid 
# 

import cv2
import numpy as np
import os
import glob

def calibrate(frames, CHECKERBOARD): 

    # store vectors of 3D points for each checkerboard image
    objpoints = []
    # Defining the world coordinates for 3D points. NOTE: z is 0 because we assume z axis is pointing out of the plane
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    objpoints.append(objp)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    #  print("Camera matrix : n")
    #  print(mtx)
    #  print("dist : n")
    #  print(dist)
    #  print("rvecs : n")
    #  print(rvecs)
    #  print("tvecs : n")
    #  print(tvecs)
