#! /usr/bin/python3
"""
This script is to:
    1. Listen to /camera/color/image_raw(sensor_msgs/Image), get the image
    2. Get point cloud data, and see the depth of each point
    3. Use YOLO-V5 to do image recognition
"""
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
from object_detection.yolo_detect import YoloDetector

class ObjectTracker(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.img_cb)
        self.aligned_pt_cloud_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pt_cloud_cb)
        self.cv_image = None
        self.pt_cloud_aligned = []
        self.img_lock = threading.Lock()
        self.pt_cloud_lock = threading.Lock()

    def pt_cloud_cb(self, msg): 
        height, width = msg.height, msg.width
        pt_cloud_gen = pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z"))
        pt_cloud_aligned = []
        for v in range(height):
            current_row = []
            for u in range(width):
                current_row.append((next(pt_cloud_gen)))
            pt_cloud_aligned.append(current_row)
        with self.pt_cloud_lock: 
            self.pt_cloud_aligned = pt_cloud_aligned

    def img_cb(self, msg): 
        with self.img_lock:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def get_coord_at_pixel(self, uv): 
        u, v = uv
        if self.cv_image is not None and self.pt_cloud_aligned: 
            with self.img_lock and self.pt_cloud_lock: 
                print(f"coord: {self.pt_cloud_aligned[v][u]}")
            cv2.circle(self.cv_image, tuple(uv), 1, color=(0, 0, 255), thickness=2)
            cv2.imshow("img", self.cv_image)
            cv2.waitKey(1)

        
if __name__ == '__main__': 
    rospy.init_node("object_tracking", anonymous=True)     #anonymous=true ensures unique node name by adding random numbers
    rate = rospy.Rate(4)
    ot = ObjectTracker()
    while not rospy.is_shutdown(): 
        # for opencv, it's (width, height)
        ot.get_coord_at_pixel((320, 240))
        rate.sleep() 


# import numpy as np
# import cv2
#
# # setup initial location of window by using two corner points
# corner_points = []
# SET_ROI = False
# PINK = (179, 102, 255)
#
# def draw_point(frame, coords): 
#     cv2.circle(frame, coords, radius=2, color=PINK, thickness=-1)
#
# def draw_box(frame, corners):
#     cv2.rectangle(frame, corners[0], corners[1], PINK, 3)
#
# def mouse_drawing(event, x_temp, y_temp, flags, params):
#     if event == cv2.EVENT_LBUTTONDOWN:
#         global corner_points
#         if not SET_ROI: 
#             corner_points.append((x_temp, y_temp))
#
# # set up video streaming
# cv2.namedWindow("object_tracking")
# cv2.setMouseCallback("object_tracking", mouse_drawing)
# cap = cv2.VideoCapture(2)
#
# while True:
#     rval, frame = cap.read()
#     # kernel = np.ones((5, 5), 'uint8')
#     # frame = cv2.erode(frame, kernel, iterations=1)
#     # frame = cv2.dilate(frame, kernel, iterations=1)
#
#     cv2.imshow("object_tracking", frame)
#     key = cv2.waitKey(20)  
#     if key == 27: # exit on ESC
#         break
