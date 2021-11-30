#! /usr/bin/env python

from __future__ import print_function

import numpy as np
import cv2 as cv
import os

TEMPLATE_IMAGE_NAME = 'not_grasped.jpg'
# IMAGE_NAME = 'not_grasped.jpg'
# IMAGE_NAME = 'grasped_f20b.jpg'
# IMAGE_NAME = 'grasped_bearing.jpg'
# IMAGE_NAME = 'grasped_axis.jpg'
# IMAGE_NAME = 'grasped_axis_2.jpg'
IMAGE_NAME = 'grasped_f20g.jpg'

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
    if show_images:
        cv.imshow('blur_median', blur_median_img)

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

def main():
    raw_test_img = get_image(IMAGE_NAME)
    cv.imshow('test_img', raw_test_img)
    processed_test_img = process(raw_test_img, show_images=False)
    cv.imshow('processed_test_img', processed_test_img)

    raw_template_img = get_image(TEMPLATE_IMAGE_NAME)
    processed_template_img = process(raw_template_img, show_images=False)
    cv.imshow('processed_template_img', processed_template_img)

    score_img = cv.matchTemplate(processed_test_img, processed_template_img, cv.TM_CCOEFF_NORMED)
    _, max_value, _, _ = cv.minMaxLoc(score_img);
    print("Matched:", round(max_value*100, 2), "%")

    print("Press 'q' to quit")
    while True:
        key = cv.waitKey(10)
        if key == ord("q"):
            break

if __name__ == "__main__":
    main()
