import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync
from utils.augmentations import Albumentations, augment_hsv, copy_paste, letterbox, mixup, random_perspective

from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory

from multiprocessing import Queue

class YoloDetector():
    """
    input_queue -----> run -----> output_queue
    """
    def __init__(self, 
                 input_queue: Queue, 
                 output_queue: Queue, 
        ):
        self.output_queue = output_queue
        self.input_queue = input_queue
        device=''  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        weights=ROOT / 'yolov5s.pt'  # model.pt path(s)
        data=ROOT / 'data/coco128.yaml'  # dataset.yaml path
        imgsz=(640, 640)  # inference size (height, width)
        conf_thres=0.25  # confidence threshold
        iou_thres=0.45  # NMS IOU threshold
        max_det=1000 # maximum detections per image
        agnostic_nms=False # class-agnostic NMS
        self.augment=False # augmented inference
        visualize=False # visualize features
        update=False # update all models
        line_thickness=3 # bounding box thickness (pixels)
        hide_labels=False # hide labels
        hide_conf=False # hide confidences
        dnn=False # use OpenCV DNN for ONNX inference

        # Load model
        print("rico device: ", device)
        self.device = select_device(device)
        self.model = DetectMultiBackend(weights, device=self.device, dnn=dnn, data=data)
        self.stride, names, pt, jit, onnx, engine = self.model.stride, self.model.names, self.model.pt, self.model.jit, self.model.onnx, self.model.engine
        self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size

        # Half, FP16 supported on limited backends with CUDA
        self.half = False & (pt or jit or onnx or engine) and self.device.type != 'cpu'  
        if pt or jit:
            self.model.model.half() if self.half else self.model.model.float()

        # Run inference
        bs = 1  # batch_size
        self.model.warmup(imgsz=(1 if pt else bs, 3, *self.imgsz), half=self.half)  # warmup
        print("imgsz: ", (1 if pt else bs, 3, *self.imgsz), "half: ", self.half)

    @torch.no_grad()
    def run(self): 
        dt = [0.0, 0.0, 0.0]
        # 1. get padded image and resize it
        img0 = self.input_queue.get(block = True)
        im = letterbox(img0, self.imgsz, stride=self.stride)[0]

        # Conversion
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)
        t1 = time_sync()
        im = torch.from_numpy(im).to(self.device)
        # RJ: Have issues if this is running on a separate process as the constructor. Probably some shared state issue
        im = im.half() if self.half else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        # # Inference
        # pred = self.model(im, augment=self.augment, visualize=False)
        # t3 = time_sync()
        # dt[1] += t3 - t2
        #
