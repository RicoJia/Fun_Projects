import cv2
from calibration import Calibrator
import argparse
LEFT=0
RIGHT=1

class StereoVideoFSM(object):
    """Finite State Machine for calibrating a stereo camera"""
    def __init__(self):
        self.__window_names = ["stereo_calibration_left", "stereo_calibration_right"]
        for window_name in self.__window_names: 
            cv2.namedWindow(window_name)
        self.vc = cv2.VideoCapture(2)
        if self.vc.isOpened():  # try to get the first frame
            self.rval = True
        else:
            self.rval = False
        print("started stereo stereo_videofsm")

    def can_get_next_frame(self): 
        return self.rval

    def get_frames(self):
        """
        :returns: frames
        """
        self.rval, big_frame = self.vc.read()
        frame_width = big_frame.shape[1]
        self.frames = [big_frame[:, 0:int(frame_width/2), :], big_frame[:, int(frame_width/2):frame_width, :]]
        return self.frames

    def get_window_names(self):
        return self.__window_names

    def show_frames_and_get_key(self):
        for i, window_name in enumerate(self.__window_names): 
            cv2.imshow(window_name, self.frames[i])
        return cv2.waitKey(20)

    def __del__(self):
        for window_name in self.__window_names: 
            cv2.destroyWindow(window_name)

stereo_videofsm = StereoVideoFSM()
window_names = stereo_videofsm.get_window_names()
left_calibrator = Calibrator(camera_name=window_names[LEFT], is_fish_eye=True)
right_calibrator = Calibrator(camera_name=window_names[RIGHT], is_fish_eye= True)
parser = argparse.ArgumentParser()
parser.add_argument("-l", required=False, help="load parameters from saved files in ~/rico_cache", action="store_true")
args = parser.parse_args()

# Single Camera calibration
if args.l: 
    left_calibrator.load_params_from_pickle()
    right_calibrator.load_params_from_pickle()

else: 
    while stereo_videofsm.can_get_next_frame():
        frames = stereo_videofsm.get_frames()
        left_calibrator.detect_and_draw_chessboard_on_frame(frames[0])
        right_calibrator.detect_and_draw_chessboard_on_frame(frames[1])
        key = stereo_videofsm.show_frames_and_get_key()
        left_calibrator.calibrate(key)
        if right_calibrator.calibrate(key): 
            break

# # Check result
# while stereo_videofsm.can_get_next_frame():
#     frames = stereo_videofsm.get_frames()
#     left_calibrator.detect_and_draw_chessboard_on_frame(frames[0])
#     right_calibrator.detect_and_draw_chessboard_on_frame(frames[1])
#     key = stereo_videofsm.show_frames_and_get_key()
#     left_calibrator.check_result(key)
#     if right_calibrator.check_result(key): 
#         break

# show depth image 
print("showing undistored frame")
while stereo_videofsm.can_get_next_frame():
    frames = stereo_videofsm.get_frames()
    frames[LEFT] = left_calibrator.undistort_frame(frames[LEFT])
    frames[RIGHT] = right_calibrator.undistort_frame(frames[RIGHT])
    key = stereo_videofsm.show_frames_and_get_key()

