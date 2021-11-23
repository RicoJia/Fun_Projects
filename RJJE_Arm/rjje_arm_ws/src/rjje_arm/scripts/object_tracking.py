import numpy as np
import cv2

# setup initial location of window by using two corner points
corner_points = []
SET_ROI = False
PINK = (179, 102, 255)

def draw_point(frame, coords): 
    cv2.circle(frame, coords, radius=2, color=PINK, thickness=-1)

def draw_box(frame, corners):
    cv2.rectangle(frame, corners[0], corners[1], PINK, 3)

def mouse_drawing(event, x_temp, y_temp, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        global corner_points
        if not SET_ROI: 
            corner_points.append((x_temp, y_temp))

# set up video streaming
cv2.namedWindow("object_tracking")
cv2.setMouseCallback("object_tracking", mouse_drawing)
cap = cv2.VideoCapture(2)

while True:
    rval, frame = cap.read()
    # kernel = np.ones((5, 5), 'uint8')
    # frame = cv2.erode(frame, kernel, iterations=1)
    # frame = cv2.dilate(frame, kernel, iterations=1)

    cv2.imshow("object_tracking", frame)
    key = cv2.waitKey(20)  
    if key == 27: # exit on ESC
        break

