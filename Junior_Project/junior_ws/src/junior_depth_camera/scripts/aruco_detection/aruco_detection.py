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

# BGR
CORNER_COLORS = ((0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 0), (255, 255, 0))
MARKER_SIZE = 0.134

class ArucoDetector:
    def __init__(self, visualize) -> None:
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.visualize = visualize
        self.camera_matrices_initialized = False
    
    def detect_aruco(self) -> bool:
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_img = np.asanyarray(color_frame.get_data())
        # TODO
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        arucoParams = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(color_img, arucoDict, parameters=arucoParams)
        print(f"ids: {ids}, corners: {corners}")
        
        if not self.camera_matrices_initialized:
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            self.camera_matrix = np.array([
                [color_intrinsics.fx, 0, color_intrinsics.ppx], 
                [0, color_intrinsics.fy, color_intrinsics.ppy],
                [0, 0, 1]
            ])
            print(self.camera_matrix)
            self.camera_matrices_initialized = True
            self.distortion_matrix = np.zeros(5)
            
        rvec_list = []
        tvec_list = []
        for marker_corners in corners:
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, self.camera_matrix, self.distortion_matrix)
            rvec_list.append(rvec)
            tvec_list.append(tvec)

        if self.visualize:
            if ids is not None:
                for rvec, tvec in zip (rvec_list, tvec_list):
                    cv2.aruco.drawAxis(color_img, self.camera_matrix, self.distortion_matrix, rvec, tvec, 0.03)  
                    print(f"position: {tvec}")
            ArucoDetector.show_aruco(color_img, corners, ids)
        
    @staticmethod
    def show_aruco(color_img, corners, ids):
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
        cv2.waitKey(2)

    def on_shutdown(self):
        self.pipeline.stop()
        
if __name__ == "__main__":
    rospy.init_node("aruco_detection")     
    rate = rospy.Rate(30)
    visualize = rospy.get_param("~visualize", False)
    print("visualize: ", visualize)
    ad = ArucoDetector(visualize)
    
    while not rospy.is_shutdown():
        ad.detect_aruco()
    
    ad.on_shutdown()