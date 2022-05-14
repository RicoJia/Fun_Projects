from yolo_detect import YoloDetector
from multiprocessing import Queue
input_queue = Queue(10)
output_queue = Queue(10)
yolo_detector = YoloDetector(input_queue = input_queue, output_queue = output_queue)
