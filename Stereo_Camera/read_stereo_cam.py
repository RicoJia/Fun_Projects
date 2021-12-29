import cv2
from calibration import Calibrator
# https://stackoverflow.com/questions/604749/how-do-i-access-my-webcam-in-python
# check a webcam ffplay /dev/video2

class VideoFSM(object):
    """Finite State Machine for calibrating a camera"""
    def __init__(self):
        self.__window_name = "calibration"
        cv2.namedWindow(self.__window_name)
        self.vc = cv2.VideoCapture(0)
        if self.vc.isOpened():  # try to get the first frame
            self.rval, self.frame = self.vc.read()
        else:
            self.rval = False
        print("started videofsm")
    
    def can_get_next_frame(self): 
        return self.rval
    
    def get_frame(self):
        """
        :returns: frame
        """
        self.rval, self.frame = self.vc.read()
        return self.frame

    def get_window_name(self):
        return self.__window_name

    def show_frame_and_get_key(self):
        cv2.imshow("calibration", self.frame)
        return cv2.waitKey(20)

    def __del__(self):
        cv2.destroyWindow(self.__window_name)

videofsm = VideoFSM()
calibrator= Calibrator()

while videofsm.can_get_next_frame():
    frame = videofsm.get_frame()
    calibrator.detect_and_draw_chessboard_on_frame(frame)
    key = videofsm.show_frame_and_get_key()
    if calibrator.calibrate(key): 
        break



