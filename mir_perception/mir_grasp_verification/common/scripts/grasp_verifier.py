#! /usr/bin/env python

from __future__ import print_function

import numpy as np
import cv2 as cv
import os

TEMPLATE_IMAGE_NAME = 'not_grasped.jpg'
# IMAGE_NAME = 'not_grasped.jpg'
# IMAGE_NAME = 'grasped_f20b.jpg'
IMAGE_NAME = 'grasped_bearing.jpg'
# IMAGE_NAME = 'grasped_axis.jpg'
# IMAGE_NAME = 'grasped_axis_2.jpg'
# IMAGE_NAME = 'grasped_f20g.jpg'

CROP_X = 440
CROP_Y = 100
CROP_HEIGHT = 280
# CROP_WIDTH = 200
CROP_WIDTH = 170
BG_REMOVAL_POLYGON = np.array([
        [0, 0],
        [40, 0],
        [5, 90],
        [10, 200],
        [42, CROP_HEIGHT],
        [0, CROP_HEIGHT]
])
HMIN = 0
HMAX = 60
SMIN = 50
SMAX = 255
VMIN = 30
VMAX = 255

use_hsv_process = False

def get_image(image_name) :
    code_dir = os.path.abspath(os.path.dirname(__file__))
    common_dir = os.path.dirname(code_dir)
    data_dir = os.path.join(common_dir, "data")
    image_path = os.path.join(data_dir, image_name)
    return cv.imread(image_path)

def process(raw_img, show_images=False):
    # cv.imshow('raw', raw_img)
    bw_raw_img = cv.cvtColor(raw_img, cv.COLOR_BGR2GRAY)
    # cv.imshow('bw_raw', bw_raw_img)
    cropped_img = bw_raw_img[CROP_Y:CROP_Y+CROP_HEIGHT, CROP_X:CROP_X+CROP_WIDTH]
    if show_images:
        cv.imshow('cropped', cropped_img)

    # thresholding
    # mean_thr_img = cv.adaptiveThreshold(cropped_img, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 15, 2)
    # cv.imshow('mean_threshold', mean_thr_img)
    adaptive_thr_img = cv.adaptiveThreshold(cropped_img, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 17, 2)
    if show_images:
        cv.imshow('adaptive_threshold', adaptive_thr_img)

    polygon_img = adaptive_thr_img.copy()
    cv.fillPoly(polygon_img, pts =[BG_REMOVAL_POLYGON], color=(0,0,0))
    if show_images:
        cv.imshow('polygon', polygon_img)

    # Blur the image for better edge detection
    # blur_img = cv.GaussianBlur(cropped_img, (5, 5), 0)
    # blur_img = cv.GaussianBlur(adaptive_thr_img, (15, 15), 0)
    # cv.imshow('blur', blur_img)
    # blur_median_img = cv.medianBlur(adaptive_thr_img, 5)
    blur_median_img = cv.medianBlur(polygon_img, 5)

    # erode and dilate
    # kernel = (3, 3)
    # erosion_img = cv.erode(adaptive_thr_img, kernel, iterations = 1)
    # cv.imshow('erosion', erosion_img)
    # opening_img = cv.morphologyEx(adaptive_thr_img, cv.MORPH_OPEN, kernel, iterations=3)
    # cv.imshow('opening', opening_img)

    # Sobel Edge Detection
    # sobelxy = cv.Sobel(src=img_blur, ddepth=cv.CV_64F, dx=1, dy=1, ksize=7) # Combined X and Y Sobel Edge Detection
    # cv.imshow('Sobel X Y using Sobel() function', sobelxy)
     
    # Canny Edge Detection
    # edges = cv.Canny(image=img_blur, threshold1=100, threshold2=200) # Canny Edge Detection
    # cv.imshow('Canny Edge', edges)
    return blur_median_img


def process_hsv(raw_img, show_images=False):
    # cv.imshow('raw', raw_img)
    hsv_raw_img = cv.cvtColor(raw_img, cv.COLOR_BGR2HSV)
    # cv.imshow('hsv_raw_img', hsv_raw_img)
    cropped_img = hsv_raw_img[CROP_Y:CROP_Y+CROP_HEIGHT, CROP_X:CROP_X+CROP_WIDTH]
    if show_images:
        cropped_rgb_img = cv.cvtColor(cropped_img, cv.COLOR_HSV2BGR)
        cv.imshow('cropped', cropped_rgb_img)

    lower = np.array([HMIN, SMIN, VMIN], np.uint8)
    upper = np.array([HMAX, SMAX, VMAX], np.uint8)
    masked_img = cv.inRange(cropped_img, lower, upper)

    return masked_img

def main():
    raw_test_img = get_image(IMAGE_NAME)
    cv.imshow('test_img', raw_test_img)
    if use_hsv_process:
        processed_test_img = process_hsv(raw_test_img, show_images=False)
    else:
        processed_test_img = process(raw_test_img, show_images=False)
    cv.imshow('processed_test_img', processed_test_img)

    bordersize = 20
    processed_test_border_img = cv.copyMakeBorder(
        processed_test_img, top=bordersize, bottom=bordersize, left=bordersize,
        right=bordersize, borderType=cv.BORDER_CONSTANT, value=0)
    cv.imshow('border', processed_test_border_img)

    raw_template_img = get_image(TEMPLATE_IMAGE_NAME)
    if use_hsv_process:
        processed_template_img = process_hsv(raw_template_img, show_images=False)
    else:
        processed_template_img = process(raw_template_img, show_images=False)
    cv.imshow('processed_template_img', processed_template_img)

    score_img = cv.matchTemplate(processed_test_border_img, processed_template_img, cv.TM_CCOEFF_NORMED)
    _, max_value, _, _ = cv.minMaxLoc(score_img);
    print("Max Matched:", round(max_value*100, 2), "%")
    cv.imshow('score_img', score_img)

    print("Press 'q' to quit")
    while True:
        key = cv.waitKey(10) & 0xFFFF
        if key == ord("q"):
            break

    cv.destroyAllWindows()

def nothing(x): # for hsv_slider function
    pass

def hsv_slider():
    raw_template_img = get_image(TEMPLATE_IMAGE_NAME)
    hsv_img = cv.cvtColor(raw_template_img, cv.COLOR_BGR2HSV)

    cv.namedWindow("image")
    cv.createTrackbar("Hmin", "image", 0, 180, nothing)
    cv.createTrackbar("Hmax", "image", 0, 180, nothing)
    cv.createTrackbar("Smin", "image", 0, 255, nothing)
    cv.createTrackbar("Smax", "image", 0, 255, nothing)
    cv.createTrackbar("Vmin", "image", 0, 255, nothing)
    cv.createTrackbar("Vmax", "image", 0, 255, nothing)

    print("Press 'q' to quit")
    while True:
        key = cv.waitKey(10) & 0xFFFF
        if key == ord("q"):
            break

        Hmin = cv.getTrackbarPos("Hmin", "image")
        Hmax = cv.getTrackbarPos("Hmax", "image")
        Smin = cv.getTrackbarPos("Smin", "image")
        Smax = cv.getTrackbarPos("Smax", "image")
        Vmin = cv.getTrackbarPos("Vmin", "image")
        Vmax = cv.getTrackbarPos("Vmax", "image")

        lower = np.array([Hmin, Smin, Vmin], np.uint8)
        upper = np.array([Hmax, Smax, Vmax], np.uint8)
        mask = cv.inRange(hsv_img, lower, upper)
        masked_img = cv.bitwise_and(raw_template_img, raw_template_img, mask=mask)

        cv.imshow("image", masked_img)

    cv.destroyAllWindows()
    pass

if __name__ == "__main__":
    main()
    # hsv_slider()
