from stereo_camera_calibrate import StereoVideoFSM
from calibration import Calibrator, StereoCalibrator
import logging
import cv2
from matplotlib import pyplot as plt
import numpy as np

LEFT=0
RIGHT=1
ESC = 27

class DepthEstimator(object): 
    def __init__(self):
        self.window_name = "Disparity Map"

    def get_disparity(self, left_frame, right_frame):
        """
        Generate a disparity map
        """

        window_size = 3
        min_disp = 16
        # In the current implementation, this parameter must be divisible by 16.
        # num_disp = 112-min_disp
        num_disp = 32
        stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
            numDisparities = num_disp,
            # Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
            blockSize = 16,
            P1 = 8*3*window_size**2,
            P2 = 32*3*window_size**2,
            disp12MaxDiff = 1,
            # Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct.
            # Normally, a value within the 5-15 range is good enough
            uniquenessRatio = 10,
            # Maximum size of smooth disparity regions to consider their noise speckles and invalidate.
            # Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
            speckleWindowSize = 100,
            # Maximum disparity variation within each connected component.
            # If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16.
            # Normally, 1 or 2 is good enough.
            # speckleRange = 32
            speckleRange = 2
        )
                
        # stereo = cv2.StereoBM_create(numDisparities=32, blockSize=31)
        gray_l = cv2.cvtColor(left_frame,cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right_frame,cv2.COLOR_BGR2GRAY)
        disparity = stereo.compute(gray_l, gray_r)
        disparity = cv2.normalize(disparity, disparity, alpha=255, beta=0, norm_type=cv2.NORM_MINMAX)
        disparity = np.uint8(disparity)
        # plt.imshow(disparity, cmap='gray')
        # plt.pause(0.005)

        cv2.imshow(self.window_name, disparity)

        return disparity

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

    # load params
    left_calibrator.load_params_from_pickle()
    right_calibrator.load_params_from_pickle()
    stereo_calibrator.load_params()

    while stereo_videofsm.can_get_next_frame():
        frames = stereo_videofsm.get_frames()

        frames[LEFT] = left_calibrator.undistort_frame(frames[LEFT])
        frames[RIGHT] = right_calibrator.undistort_frame(frames[RIGHT])
        key = stereo_videofsm.show_frames_and_get_key()
        depth_estimator.get_disparity(frames[LEFT], frames[RIGHT])
        if depth_estimator.exit(key): 
            break
