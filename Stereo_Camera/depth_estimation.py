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
    def __init__(self):
        self.window_name = "Disparity Map"

        # for disparity img
        self.window_size = 15
        self.min_disp = 0
        # In the current implementation, this parameter must be divisible by 16.
        # num_disp = 112-min_disp
        self.num_disp = 32
        self.block_size = 16
        self.disp12MaxDiff = 24
        self.uniquenessRatio = 23
        self.speckleWindowSize = 90
        self.speckleRange = 22
        self.re_reprojected_window_name = "re_reprojected image"

        fig = plt.figure(figsize=(12, 12))
        self.ax = fig.gca(projection='3d')

    def update(self, val=0):
        self.window_size = int(cv2.getTrackbarPos('window_size', 'disparity'))
        self.uniquenessRatio = int(cv2.getTrackbarPos('uniquenessRatio', 'disparity'))
        self.speckleWindowSize = int(cv2.getTrackbarPos('speckleWindowSize', 'disparity'))
        self.speckleRange = int(cv2.getTrackbarPos('speckleRange', 'disparity'))
        self.disp12MaxDiff = int(cv2.getTrackbarPos('disp12MaxDiff', 'disparity'))

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
            speckleRange = self.speckleRange
        )
                
        # stereo = cv2.StereoBM_create(numDisparities=32, blockSize=31)
        gray_l = cv2.cvtColor(left_frame,cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right_frame,cv2.COLOR_BGR2GRAY)
        disparity = stereo.compute(gray_l, gray_r)

        self.normalized_disparity = np.uint8(cv2.normalize(disparity, disparity, alpha=255, beta=0, norm_type=cv2.NORM_MINMAX))

        return disparity

    def get_3d_points(self, disparity, Q):
        #reporject image will keep the same size, but will have inf if they are not available
        points_3d = cv2.reprojectImageTo3D(disparity, Q)
        return points_3d

    def display_depth_on_img(self, point_pos, frame): 
        # display the mid point's depth 
        cln, row = point_pos.shape[:2]
        print((int(row/2), int(cln/2)))
        mid_xyz = point_pos[int(row/2), int(cln/2), :]
        cv2.circle(frame, (int(row/2), int(cln/2)), 3, (0, 255, 255), -1)
        cv2.imshow(self.window_name, self.normalized_disparity)
        print("pt 3d: ", mid_xyz)
        print("disp val: ", self.normalized_disparity[int(row/2), int(cln/2)])
    # def show_3d_point_cloud(self, point_pos, BGR):
    #     """
    #     Takes in 3d point and color, show it as point cloud
    #     """
    #     mask = np.logical_and(point_pos > point_pos.min(), point_pos < point_pos.max())
    #     point_pos = point_pos[mask].reshape(-1, 3)
    #     self.ax.scatter(point_pos[:, 0], point_pos[:, 1], point_pos[:,2])
    #     plt.pause(0.01)


    # def project_3d_point_to_cam(self, point_pos, BGR, mtx): 
    #     mask = np.logical_and(point_pos > point_pos.min(), point_pos < point_pos.max()) 
    #     point_pos = point_pos[mask].reshape(-1, 3)
    #     BGR_cp = BGR[mask].reshape(-1, 3)
    #     projected_pts, _ = cv2.projectPoints(point_pos, np.identity(3),
    #               np.array([0., 0., 0.]),
    #               mtx, np.array([0., 0., 0., 0.]))
    #
    #     blank_img = np.zeros(BGR_cp.shape)
    #     for i, pt in enumerate(projected_pts):
    #         # use the BGR format to match the original image type
    #         col = (BGR[i, 2], BGR[i, 1], BGR[i, 0])
    #         print (col)
    #         # cv2.circle(blank_img, (pt[0, 0], pt[0, 1]), 1, col)
    #
    #     cv2.imshow(self.re_reprojected_window_name, blank_img)


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
    depth_estimator = DepthEstimator()

    # #Track bar
    cv2.namedWindow('disparity')
    cv2.createTrackbar('speckleRange', 'disparity', depth_estimator.speckleRange, 50, depth_estimator.update)
    cv2.createTrackbar('window_size', 'disparity', depth_estimator.window_size, 21, depth_estimator.update)
    cv2.createTrackbar('speckleWindowSize', 'disparity', depth_estimator.speckleWindowSize, 200, depth_estimator.update)
    cv2.createTrackbar('uniquenessRatio', 'disparity', depth_estimator.uniquenessRatio, 50, depth_estimator.update)
    cv2.createTrackbar('disp12MaxDiff', 'disparity', depth_estimator.disp12MaxDiff, 250, depth_estimator.update)

    # load params
    left_calibrator.load_params_from_pickle()
    right_calibrator.load_params_from_pickle()
    stereo_calibrator.load_params()

    while stereo_videofsm.can_get_next_frame():
        frames = stereo_videofsm.get_frames()

        frames[LEFT] = left_calibrator.undistort_frame(frames[LEFT])
        frames[RIGHT] = right_calibrator.undistort_frame(frames[RIGHT])
        disparity = depth_estimator.get_disparity(frames[LEFT], frames[RIGHT])
        point_pos = depth_estimator.get_3d_points(disparity, stereo_calibrator.stereo_camera_params["Q"])
        # Rectification was done in left -> right
        # depth_estimator.display_depth_on_img(point_pos, depth_estimator.normalized_disparity)
        # depth_estimator.show_3d_point_cloud(point_pos, frames[RIGHT])
        # depth_estimator.project_3d_point_to_cam(point_pos, frames[RIGHT], right_calibrator.params["mtx"])
        key = stereo_videofsm.show_frames_and_get_key()
        if depth_estimator.exit(key): 
            break
