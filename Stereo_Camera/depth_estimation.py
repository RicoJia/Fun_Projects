from stereo_camera_calibrate import StereoVideoFSM
from calibration import Calibrator, StereoCalibrator
import logging
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import calendar
import argparse
import os
import open3d as o3d

LEFT=0
RIGHT=1
ESC = 27
ENTER = 13

class DepthEstimator(object): 
    def __init__(self, T, Q):
        self.window_name = "Disparity Map"
        self.baseline = abs(T[0])
        self.Q = Q
        # for disparity img
        self.window_size = 10
        self.min_disp = 1
        # In the current implementation, this parameter must be divisible by 16.
        # num_disp = 112-min_disp
        self.num_disp = 32
        self.block_size = 16
        self.disp12MaxDiff = 24
        self.uniquenessRatio = 6
        self.speckleWindowSize = 66
        self.speckleRange = 25
        self.re_reprojected_window_name = "re_reprojected image"
        fig = plt.figure(figsize=(12, 12))
        self.ax = fig.gca(projection='3d')
        self.launch_trackbar()

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.frame_store_dir = os.path.join(current_dir, "images_stereo")


    def update(self, val=0):
        self.window_size = int(cv2.getTrackbarPos('window_size', 'disparity'))
        self.uniquenessRatio = int(cv2.getTrackbarPos('uniquenessRatio', 'disparity'))
        self.speckleWindowSize = int(cv2.getTrackbarPos('speckleWindowSize', 'disparity'))
        self.speckleRange = int(cv2.getTrackbarPos('speckleRange', 'disparity'))
        self.disp12MaxDiff = int(cv2.getTrackbarPos('disp12MaxDiff', 'disparity'))
        self.min_disp = int(cv2.getTrackbarPos('min_disp', 'disparity'))
        # In the current implementation, this parameter must be divisible by 16.
        # num_disp = 112-min_disp
        self.num_disp = (int(cv2.getTrackbarPos('num_disp', 'disparity')) + 1)* 16
        self.block_size = int(cv2.getTrackbarPos('block_size', 'disparity')) * 16

    def get_and_show_depth_image(self, frames):
        """
        Generate a disparity map, and display the mid point's depth
        :param frames - frames[LEFT], frames[RIGHT]
        """
        left_frame, right_frame = frames[LEFT], frames[RIGHT]
        stereo = cv2.StereoSGBM_create(
            minDisparity = self.min_disp,
            numDisparities = self.num_disp,
            # Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
            blockSize = self.block_size,
            P1 = 8*1*self.window_size**2,
            P2 = 32*1*self.window_size**2,
            disp12MaxDiff = self.disp12MaxDiff,
            # Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct.
            # Normally, a value within the 5-15 range is good enough
            uniquenessRatio = self.uniquenessRatio,
            # Maximum size of smooth disparity regions to consider their noise speckles and invalidate.
            # Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
            speckleWindowSize = self.speckleRange,
            # Maximum disparity variation within each connected component.
            # If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16.
            # Normally, 1 or 2 is good enough.
            # speckleRange = 32
            speckleRange = self.speckleRange, 
            mode=cv2.STEREO_SGBM_MODE_HH
        )
                
        # stereo = cv2.StereoBM_create(numDisparities=32, blockSize=31)
        gray_l = cv2.cvtColor(left_frame,cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right_frame,cv2.COLOR_BGR2GRAY)
        disparity_img = stereo.compute(gray_l, gray_r)
        
        self.normalized_disparity = np.uint8(cv2.normalize(disparity_img, None, alpha=255, beta=0, norm_type=cv2.NORM_MINMAX))

        # Note: opencv implementation multiplies the whole thing with 16 for accuracy. Need to tune this down.
        disparity_img = disparity_img.astype(np.float32)/16.0

        #reporject image will keep the same size, but will have inf if they are not available
        points_3d = cv2.reprojectImageTo3D(disparity_img, self.Q)
        # display the mid point's depth 
        cln, row = points_3d.shape[:2]
        mid_xyz = points_3d[int(row/2), int(cln/2), :]
        cv2.circle(self.normalized_disparity, (int(row/2), int(cln/2)), 3, (0, 255, 255), -1)
        print("pt 3d: ", mid_xyz)
        cv2.imshow(self.window_name, self.normalized_disparity)
        return disparity_img, points_3d
    
    def visualize_3d_point_cloud(self, points_3d):
        """
        Visualize the 3d point cloud in open3D
        """
        # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
        pcd = o3d.geometry.PointCloud()
        points_3d = points_3d.reshape(-1, 3)
        points_3d = points_3d[(~np.isinf(points_3d).any(axis=1))]
        #TODO
        print(points_3d)
        pcd.points = o3d.utility.Vector3dVector(points_3d)
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0,0,0])
        o3d.visualization.draw_geometries([pcd, mesh_frame])

    def exit(self, key):
        """
        If key is ESC they exit
        """
        if key == ESC: 
            logging.info("Esc detected, exited depth estimation")
            return True
        else: 
            return False
    
    def save_frames_if_necessary(self, key, frames): 
        """
        If ENTER is hit, left and right frames will be saved under ~/images_stereo
        File Names will be: left_frame_TIMESTAMP.png
        """
        if key == ENTER: 
            if not os.path.isdir(self.frame_store_dir): 
                os.mkdir(self.frame_store_dir)
            tm = calendar.timegm(time.gmtime()) 
            cv2.imwrite(f"{self.frame_store_dir}/left_frame_{tm}.png", frames[LEFT])
            cv2.imwrite(f"{self.frame_store_dir}/right_frame_{tm}.png", frames[RIGHT])
            logging.info("Left and right frames have been saved.")

    def load_frames(self): 
        ls = os.listdir(self.frame_store_dir)
        left_frame_names = list(filter(lambda x: "left" in x, ls))
        right_frame_names = list(filter(lambda x: "right" in x, ls))
        frames_list = []
        for left_name, right_name in zip(left_frame_names, right_frame_names): 
            left_frame = cv2.imread(os.path.join(self.frame_store_dir, left_name))
            right_frame = cv2.imread(os.path.join(self.frame_store_dir, right_name))
            frames = [left_frame, right_frame]
            frames_list.append(frames)
        logging.info(f"{len(frames_list)} sets of left and right stereo images have been loaded")
        return frames_list


    def launch_trackbar(self):
        """
        Launch track bar for tuning
        """
        # #Track bar
        cv2.namedWindow('disparity')
        cv2.createTrackbar('speckleRange', 'disparity', self.speckleRange, 50, self.update)
        cv2.createTrackbar('window_size', 'disparity', self.window_size, 21, self.update)
        cv2.createTrackbar('speckleWindowSize', 'disparity', self.speckleWindowSize, 200, self.update)
        cv2.createTrackbar('uniquenessRatio', 'disparity', self.uniquenessRatio, 50, self.update)
        cv2.createTrackbar('disp12MaxDiff', 'disparity', self.disp12MaxDiff, 250, self.update)
        cv2.createTrackbar('min_disp', 'disparity', self.min_disp, 250, self.update)
        cv2.createTrackbar('num_disp', 'disparity', int(self.num_disp/32), 10, self.update)
        cv2.createTrackbar('block_size', 'disparity', int(self.block_size/32), 10, self.update)


class Parser(object):
    """abstraction for arg_parser"""
    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("-r", required=False, help="save stereo images. Regular mode is to load images from $(pwd)/images_stereo", action="store_true")
        self.args = parser.parse_args()
        

if __name__ == "__main__": 
    p = Parser()
    stereo_videofsm = StereoVideoFSM()
    window_names = stereo_videofsm.get_window_names()
    left_calibrator = Calibrator(camera_name=window_names[LEFT], is_fish_eye=False)
    right_calibrator = Calibrator(camera_name=window_names[RIGHT], is_fish_eye= False)
    stereo_calibrator = StereoCalibrator(window_name="stereo_calibrator", left_camera_params=left_calibrator.params, right_camera_params=right_calibrator.params)

    # load params
    left_calibrator.load_params_from_pickle()
    right_calibrator.load_params_from_pickle()
    stereo_calibrator.load_params()

    depth_estimator = DepthEstimator(T = stereo_calibrator.stereo_camera_params["T"], Q = stereo_calibrator.stereo_camera_params["Q"])
    
    if p.args.r: 
        print("You can record multiple pictures by hitting Enter. Hit Esc to quit")
        while stereo_videofsm.can_get_next_frame():
            frames = stereo_videofsm.get_frames()
            frames[LEFT] = left_calibrator.undistort_frame(frames[LEFT])
            frames[RIGHT] = right_calibrator.undistort_frame(frames[RIGHT])
            depth_estimator.get_and_show_depth_image(frames)
            key = stereo_videofsm.show_frames_and_get_key()
            depth_estimator.save_frames_if_necessary(key, frames)
            if depth_estimator.exit(key): 
                break

    frames_list = depth_estimator.load_frames()
    for frames in frames_list: 
        _, points_3d = depth_estimator.get_and_show_depth_image(frames)
        cv2.waitKey(0)
        depth_estimator.visualize_3d_point_cloud(points_3d)
