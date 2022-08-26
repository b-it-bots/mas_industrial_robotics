# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
from .utils.general import (check_img_size, non_max_suppression, print_args, scale_coords)
import cv2
from .utils.torch_utils import select_device
from .models.common import DetectMultiBackend
import argparse
import os
import sys
from pathlib import Path
import numpy as np

import torch
import torch.backends.cudnn as cudnn

import roslib


FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative


class_names = ['R20', 'tennis_ball', 'F20_20_B', 'pringles', 'insulation_tape', 'bearing_box', 'sponge', 'towel',
               'screw_driver', 'spoon', 'motor', 'dishwasher_soap', 'S40_40_B', 'brown_box', 'cup', 'axis',
               'S40_40_G', 'bracket', 'eye_glasses', 'toothbrush']


def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)


result = {}


@torch.no_grad()
def run(
        weights='yolov5m.pt',  # model.pt path(s)
        source='dummy_images',  # file/dir/URL/glob, 0 for webcam
        data='dummy_data.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        half=False,  # use FP16 half-precision inference
):
    # source = str(source)

    weights = os.path.join(weights, 'best.pt')

    # Directories
    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Dataloader
    bs = 1  # batch_size
    # Run inference
    # im0 = cv2.imread(source, 1)
    im0 = source
    img_size = 640
    stride = 32
    auto = True
    img = letterbox(im0, img_size, stride=stride, auto=auto)[0]

    # Convert
    img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    img = np.ascontiguousarray(img)
    model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
    im = torch.from_numpy(img).to(device)
    im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
    im /= 255  # 0 - 255 to 0.0 - 1.0
    if len(im.shape) == 3:
        im = im[None]  # expand for batch dim

    # Inference
    pred = model(im)

    # NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

    # Second-stage classifier (optional)
    # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

    # Process predictions
    bbox = []
    confidence = []
    classes = []
    for i, det in enumerate(pred):  # per image
        im0 = im0.copy()  # raw value need to change

        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        # print('Detection: ',det)
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()
            # Write results
            for *xyxy, conf, cls in det:
                # print('xyxy',xyxy)
                # print('Class : ',class_names[int(cls.item())], 'Conf : ', conf)
                bbox.append([x.item() for x in xyxy])
                confidence.append(conf.item())
                classes.append(class_names[int(cls.item())])

    result['boxes'] = bbox
    result['labels'] = classes
    result['scores'] = confidence

    # print(result)

    return result


def main():

    model_file = os.path.join(roslib.packages.get_pkg_dir("mir_rgb_object_recognition_models"),
                              'common', 'models', 'yolov5', 'atwork_realdata_combined', "best.pt")

    inference_image_file = os.path.join(roslib.packages.get_pkg_dir("mir_object_recognition"),
                                        'common', 'src', 'rgb_object_recognition', 'yolov5', 'data',
                                        'images', 'zidane.jpg')

    data_yaml = os.path.join(roslib.packages.get_pkg_dir("mir_object_recognition"),
                             'common', 'src', 'rgb_object_recognition', "data_for_detect.yaml")

    model_path = model_file

    predictions = run(weights=model_path,
                      data=data_yaml,
                      source=inference_image_file)


if __name__ == "__main__":
    main()
