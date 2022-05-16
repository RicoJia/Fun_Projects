from yolo_detect import YoloDetector
from multiprocessing import Queue
from multiprocessing import  Process
input_queue = Queue(maxsize = 10)
output_queue = Queue(maxsize = 10)
path = "data/images/"

import signal
import os
import cv2
all_files = os.listdir(path)
for file_name in all_files: 
    file_path = os.path.join(path, file_name)
    print(file_path)
    img = cv2.imread(file_path, cv2.IMREAD_COLOR)
    input_queue.put(img)

# Need to launch the yolo detector and run together in a separate process
def process_func(input_queue, output_queue): 
    yolo_detector = YoloDetector(input_queue = input_queue, output_queue = output_queue)
    yolo_detector.run()
    print("finished run")
proc = Process(target=process_func, args=(input_queue, output_queue))
proc.start()

import time
time.sleep(1)
while not output_queue.empty():
    print("2")
    im0 = output_queue.get(block=True, timeout=0.1)
    cv2.imshow("asdf", im0)
    cv2.waitKey(0)  # 1 millisecond

os.kill(proc.pid, signal.SIGUSR2)
proc.join()

# clear queue 
while not output_queue.empty(): 
    output_queue.get(block=True, timeout=0.1)
while not input_queue.empty(): 
    input_queue.get(block=True, timeout=0.1)

# cv2.imshow("sdf", img0)
# cv2.waitKey(0)
# print("finish run")
