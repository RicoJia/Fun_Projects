import numpy as np
import cv2

# keyboard control stuff
from pynput.keyboard import Key, Controller
import queue

keyboard = Controller()

cv2.namedWindow("preview")
cap = cv2.VideoCapture(0)

# take first frame of the video
ret, frame = cap.read()

corner_points = []
SET_ROI = False
def mouse_drawing(event, x_temp, y_temp, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        global corner_points, SET_ROI, track_window
        if not SET_ROI: 
            corner_points.append(np.array([x_temp, y_temp], dtype=int))
            if len(corner_points) == 2: 
                center = ((corner_points[0] + corner_points[1])/2).astype(int)
                x, y = center[0], center[1]
                abs_diff = np.abs(corner_points[0] - corner_points[1])
                height, width = abs_diff[0], abs_diff[1]
                # setup initial location of window
                track_window = (x, y ,width, height)
                #TODO
                print(f"corner_pts: {corner_points}")
                print(f"diff: {corner_points[0] - corner_points[1]}")
                SET_ROI = True

PINK = (179, 102, 255)
def draw_point(frame, coords): 
    cv2.circle(frame, tuple(coords), radius=2, color=PINK, thickness=2)

cv2.setMouseCallback("preview", mouse_drawing)

while (SET_ROI==False): 
    rval, frame = cap.read()
    for pt in corner_points:
        draw_point(frame, pt)
    cv2.imshow("preview", frame)
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break


x, y, width, height = track_window
roi = frame[y-height:y+height, x-width : x+width]
hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv_roi, np.array((0., 60., 32.)), np.array((180., 255., 255)))
roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
cv2.normalize(roi_hist, roi_hist, 0, 255,cv2.NORM_MINMAX)
# Setup the termination criteria, either 10 iteration or move by atleast 1 pt
term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
cv2.imshow('roi',roi)

x_window, y_window, w_window, h_window = cv2.getWindowImageRect('preview')

FRAME_INTERVAL = 30     #ms
LEFT_THRESH = int (w_window * 0.75)           #remember left should be larger
RIGHT_THRESH = int (w_window * 0.35)

while(1):
    ret, frame = cap.read()
    if ret == True:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)
        # apply meanshift to get the new location
        ret, track_window = cv2.CamShift(dst, track_window, term_crit)

        # Draw it on image
        pts = cv2.boxPoints(ret)
        print(pts)
        pts = np.int0(pts)
        final_image = cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
        #x,y,w,h = track_window
        #final_image = cv2.rectangle(frame, (x,y), (x+w, y+h), 255, 3)

        # mean
        x_mean = np.mean(pts, axis=0)[0]
        print("mean: ", x_mean)
        if (x_mean > LEFT_THRESH): 
            keyboard.press('3')
            keyboard.release('3')
            print("l")
        elif x_mean < RIGHT_THRESH: 
            keyboard.press('7')
            keyboard.release('7')
            print("r")



        # cv2.imshow('dst', dst)
        cv2.imshow('preview',final_image)


        k = cv2.waitKey(FRAME_INTERVAL) & 0xff
        if k == 27:
            break
    else:
        break
