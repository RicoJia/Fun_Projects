import cv2
from calibration import calibrate
# https://stackoverflow.com/questions/604749/how-do-i-access-my-webcam-in-python
# check a webcam vlc v4l2:///dev/video2

cv2.namedWindow("calibration")
vc = cv2.VideoCapture(0)

if vc.isOpened():  # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

# Defining the dimensions of checkerboard TODO
CHECKERBOARD = (7, 4)   #Convention: (x, y)
# Teling the cv2 that we want to stop once max_iter, or convergence metrics reaches some small value. 
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# store vectors of 2D points for each checkerboard image
imgpoints = []
frames = []
paused = False

# if a chessboard is detected and we hit enter, then video pauses and calibrate 
# To save the parameters, hit y; to do another calibration session, hit n. 
while rval:
    if not paused: 
        rval, frame = vc.read()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret == True: 
            print("hehe")
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            # refining pixel coordinates for given 2d points.
            imgpoints.append(corners2)
            frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)

    cv2.imshow("calibration", frame)
    key = cv2.waitKey(20)
    if key == 13: 
        frames.append(gray)
    # elif key == 27:  # exit on ESC
    #     break


cv2.destroyWindow("calibration")

# calibration(frames)
