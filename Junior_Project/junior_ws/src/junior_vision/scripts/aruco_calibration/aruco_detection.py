#! /usr/bin/python3
"""
This script is to: 
    1. Query Intel Realsense D415 for RGB and intrisics
    2. Detect Aruco Markers (4x4, 100 members in the Aruco Dictionary)
    3. Find Aruco Markers w.r.t Camera frame of Intel Real Sense
Inspired by: https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python
"""
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
from typing import Tuple

# BGR
CORNER_COLORS = ((0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 0), (255, 255, 0))
DEFAULT_MARKER_SIZE = 0.134

class ArucoDetector:
    def __init__(self, visualize, marker_size = DEFAULT_MARKER_SIZE) -> None:
        self.visualize = visualize
        self.marker_size = marker_size
        self.camera_matrices_initialized = False
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
    
    def detect_aruco(self) -> Tuple[np.ndarray, list, np.ndarray]:
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_img = np.asanyarray(color_frame.get_data())
        # TODO
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        arucoParams = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(color_img, arucoDict, parameters=arucoParams)
        if not self.camera_matrices_initialized:
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            self.camera_matrix = np.array([
                [color_intrinsics.fx, 0, color_intrinsics.ppx], 
                [0, color_intrinsics.fy, color_intrinsics.ppy],
                [0, 0, 1]
            ])
            rospy.logdebug(self.camera_matrix)
            self.camera_matrices_initialized = True
            self.distortion_matrix = np.zeros(5)
        return color_img, corners, ids

    def estimate_aruco_poses(self, color_img: np.ndarray, corners: list, ids: np.ndarray) -> Tuple[np.ndarray, list, np.ndarray]:
        rospy.logdebug(f"ids: {ids}, corners: {corners}")
        rvec_list = []
        tvec_list = []
        for marker_corners in corners:
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners, self.marker_size, self.camera_matrix, self.distortion_matrix)
            rvec_list.append(rvec)
            tvec_list.append(tvec)

        if self.visualize:
            self.show_aruco(color_img, corners, ids, rvec_list, tvec_list)
        return ids, rvec_list, tvec_list
        
    def show_aruco(self, color_img: np.ndarray, corners: list, ids: np.ndarray, rvec_list: list = [], tvec_list: list = [], show_static_img = False) -> int:
        if rvec_list:
            for rvec, tvec in zip (rvec_list, tvec_list):
                cv2.aruco.drawAxis(color_img, self.camera_matrix, self.distortion_matrix, rvec, tvec, 0.03)  
                rospy.logdebug(f"position: {tvec}")
        # Corners are all in top-left, top-right, bottom-right, and bottom-left order)
        if ids is not None:
            ids = ids.flatten()
            for id, marker_corners in zip(ids, corners):
                marker_corners = marker_corners.reshape((4,2)).astype(int)
                for color, marker_corner in zip(CORNER_COLORS, marker_corners):
                    cv2.circle(color_img, tuple(marker_corner), 4, color, -1)
                top_left = marker_corner[0:]
                cv2.putText(color_img, str(id),tuple(top_left - np.array([0, 10], int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Aruco Detection", color_img)
        timeout = 0 if show_static_img else 1 
        key_pressed = cv2.waitKey(timeout)
        return key_pressed

    def on_shutdown(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        
if __name__ == "__main__":
    rospy.init_node("aruco_detection")     
    rate = rospy.Rate(30)
    visualize = rospy.get_param("~visualize", True)
    rospy.logdebug("visualize: ", visualize)
    ad = ArucoDetector(visualize, 0.036)
    
    while not rospy.is_shutdown():
        color_img, corners, ids = ad.detect_aruco()
        ad.estimate_aruco_poses(color_img, corners, ids)
        rate.sleep()
    
    ad.on_shutdown()