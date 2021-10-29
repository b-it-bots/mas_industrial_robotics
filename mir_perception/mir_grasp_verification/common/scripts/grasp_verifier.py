#! /usr/bin/env python

from __future__ import print_function

import cv2
import os

IMAGE_NAME = 'not_grasped.jpg'
# IMAGE_NAME = 'grasped_f20b.jpg'
# IMAGE_NAME = 'grasped_bearing.jpg'

def get_image() :
    code_dir = os.path.abspath(os.path.dirname(__file__))
    common_dir = os.path.dirname(code_dir)
    data_dir = os.path.join(common_dir, "data")
    image_path = os.path.join(data_dir, IMAGE_NAME)
    return cv2.imread(image_path)

def main():
    raw_img = get_image()
    # cv2.imshow('raw', raw_img)
    bw_raw_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('bw_raw', bw_raw_img)
    # cv2.waitKey(0)
    cropped_img = bw_raw_img[100:380, 440:640]
    cv2.imshow('cropped', cropped_img)
    # cv2.waitKey(0)
    # Blur the image for better edge detection
    img_blur = cv2.GaussianBlur(cropped_img, (5, 5), 0)
    # img_blur = cv2.GaussianBlur(cropped_img, (15, 15), 0)
    cv2.imshow('blur', img_blur)
     
    # Sobel Edge Detection
    sobelxy = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=7) # Combined X and Y Sobel Edge Detection
    cv2.imshow('Sobel X Y using Sobel() function', sobelxy)
     
    # Canny Edge Detection
    edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200) # Canny Edge Detection
    # edges_1 = cv2.Canny(image=img_blur, threshold1=50, threshold2=200) # Canny Edge Detection
    # edges_2 = cv2.Canny(image=img_blur, threshold1=100, threshold2=250) # Canny Edge Detection
    # edges_3 = cv2.Canny(image=img_blur, threshold1=150, threshold2=200) # Canny Edge Detection
    # edges_4 = cv2.Canny(image=img_blur, threshold1=100, threshold2=150) # Canny Edge Detection
    # Display Canny Edge Detection Image
    cv2.imshow('Canny Edge', edges)
    # cv2.imshow('Canny Edge 1', edges_1)
    # cv2.imshow('Canny Edge 2', edges_2)
    # cv2.imshow('Canny Edge 3', edges_3)
    # cv2.imshow('Canny Edge 4', edges_4)
    cv2.waitKey(0)

if __name__ == "__main__":
    main()
