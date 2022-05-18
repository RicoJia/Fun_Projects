#!/usr/bin/python3
"""
- Inputs of the module: 
    - image with aruco markers
    - intrinsics of the camera (pinhole model)
- Outputs: 
    - {marker_id: pose (4x4 np.ndarray) ...}
- Similar Project: https://github.com/hm7455/Exhibition-hall-robot_vision/blob/main/aruco%20_mark/detect_aruco_dynamic_realsense_pub.py
"""
import numpy as np
from typing import Dict
import cv2

class ArucoExtrinsics(): 
    def __init__(self):
        pass 
    def calibrate(self, img: np.ndarray): 
        # get extrinsics 
        # get intrinsics
        # detect aruco markers from camera view
        # aruco_detections = {marker_id: pose}
        aruco_detections = self._detect_aruco_markers(img)
        for corners in aruco_detections.values(): 
            for corner in corners[0]: 
                cv2.circle(img, tuple(corner.astype(int)), 3, color=(0, 0, 255), thickness=2)

        self._estimate_pose(aruco_detections)

    def _detect_aruco_markers(self, img: np.ndarray): 
        """
        rejected: A list of potential markers that were found but ultimately rejected due to the inner code of the marker being unable to be parsed (visualizing the rejected markers is often useful for debugging purposes)
        """
        # load aruco dictionary
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        # Define the ArUco detection parameters using 
        parameters =  cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters)
        print(corners)
        return {ids[i][0]: corner_list for i, corner_list in enumerate(corners)}

    def _estimate_pose(self, aruco_detections: Dict): 
        pass

#TODO
if __name__ == "__main__":
    img = cv2.imread("data/images/aruco_2.jpg", cv2.IMREAD_COLOR)
    ae = ArucoExtrinsics()
    ae.calibrate(img)
    cv2.imshow("img", img)
    cv2.waitKey(0)
