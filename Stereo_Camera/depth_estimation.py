from stereo_camera_calibrate import StereoVideoFSM
from calibration import Calibrator, StereoCalibrator
import logging
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

LEFT=0
RIGHT=1
ESC = 27

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

    def get_disparity(self, left_frame, right_frame):
        """
        Generate a disparity map
        """

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
        disparity = stereo.compute(gray_l, gray_r)
        
        self.normalized_disparity = np.uint8(cv2.normalize(disparity, None, alpha=255, beta=0, norm_type=cv2.NORM_MINMAX))

        return disparity

    def get_3d_points(self, disparity):
        #reporject image will keep the same size, but will have inf if they are not available
        points_3d = cv2.reprojectImageTo3D(disparity, self.Q)
        return points_3d

    def get_depth(self, disparity_val): 
        f = self.Q[2, 3]
        return f * self.baseline/ disparity_val

    def display_depth_on_disparity_img(self, point_pos, disparity_img): 
        """
        Display the mid point's depth
        """
        # display the mid point's depth 
        cln, row = point_pos.shape[:2]
        mid_xyz = point_pos[int(row/2), int(cln/2), :]
        cv2.circle(self.normalized_disparity, (int(row/2), int(cln/2)), 3, (0, 255, 255), -1)
        cv2.imshow(self.window_name, self.normalized_disparity)
        # Note: opencv implementation multiplies the whole thing with 16 for accuracy. Need to tune this down.
        disparity_img = disparity_img.astype(np.float32)/16.0

        print("pt 3d: ", self.get_depth(disparity_img[int(row/2), int(cln/2)]))
        print(f"disp: {disparity_img[int(row/2), int(cln/2)]}, f:{self.Q[2,3]}, baseline: {self.baseline}")

    def exit(self, key):
        """
        If key is ESC they exit
        """
        if key == ESC: 
            logging.info("Esc detected, exited depth estimation")
            return True
        else: 
            return False
        

if __name__ == "__main__": 
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
    # #Track bar
    cv2.namedWindow('disparity')
    cv2.createTrackbar('speckleRange', 'disparity', depth_estimator.speckleRange, 50, depth_estimator.update)
    cv2.createTrackbar('window_size', 'disparity', depth_estimator.window_size, 21, depth_estimator.update)
    cv2.createTrackbar('speckleWindowSize', 'disparity', depth_estimator.speckleWindowSize, 200, depth_estimator.update)
    cv2.createTrackbar('uniquenessRatio', 'disparity', depth_estimator.uniquenessRatio, 50, depth_estimator.update)
    cv2.createTrackbar('disp12MaxDiff', 'disparity', depth_estimator.disp12MaxDiff, 250, depth_estimator.update)
    cv2.createTrackbar('min_disp', 'disparity', depth_estimator.min_disp, 250, depth_estimator.update)
    cv2.createTrackbar('num_disp', 'disparity', int(depth_estimator.num_disp/32), 10, depth_estimator.update)
    cv2.createTrackbar('block_size', 'disparity', int(depth_estimator.block_size/32), 10, depth_estimator.update)
    while stereo_videofsm.can_get_next_frame():
        frames = stereo_videofsm.get_frames()

        frames[LEFT] = left_calibrator.undistort_frame(frames[LEFT])
        frames[RIGHT] = right_calibrator.undistort_frame(frames[RIGHT])
        disparity = depth_estimator.get_disparity(frames[LEFT], frames[RIGHT])
        # Rectification was done in left -> right
        point_pos = depth_estimator.get_3d_points(disparity)
        depth_estimator.display_depth_on_disparity_img(point_pos, disparity)
        key = stereo_videofsm.show_frames_and_get_key()
        if depth_estimator.exit(key): 
            break
