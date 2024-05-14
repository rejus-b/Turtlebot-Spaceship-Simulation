import cv2
import threading 
import numpy as np 
import rclpy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.exceptions import ROSInterruptException
import signal

green_found = 1
red_found = 2
sensitivity = 15

def detect_button(image):
    hsv_red_lower = np.array([0 - sensitivity, 100, 100])
    hsv_red_upper = np.array([0 + sensitivity, 255, 255])
    
    hsv_green_lower = np.array([60 - sensitivity, 100, 100])
    hsv_green_upper = np.array([60 + sensitivity, 255, 255])

    # Convert the rgb image into a hsv image
    Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but a particular colour using the cv2.inRange() method
    green_mask = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
    red_mask = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)
    
    masked_image_green = cv2.bitwise_and(image, image, mask=green_mask)
    masked_image_red = cv2.bitwise_and(image, image, mask=red_mask)

    gray_green = cv2.cvtColor(masked_image_green, cv2.COLOR_BGR2GRAY)
    blurred_green = cv2.GaussianBlur(gray_green, (9, 9), 2)
    
    gray_red = cv2.cvtColor(masked_image_red, cv2.COLOR_BGR2GRAY)
    blurred_red = cv2.GaussianBlur(gray_red, (9, 9), 2)
    
    circles_green = cv2.HoughCircles(blurred_green, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                               param1=50, param2=30, minRadius=0, maxRadius=0)
    
    circles_red = cv2.HoughCircles(blurred_red, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                               param1=50, param2=30, minRadius=0, maxRadius=0)
    
    green_image = cv2.bitwise_and(image, image, mask=green_mask)
    red_image = cv2.bitwise_and(image, image, mask=red_mask)
    gr_image = cv2.bitwise_or(green_image,red_image)
    
    cv2.namedWindow('button_detector',cv2.WINDOW_NORMAL) 
    cv2.imshow('button_detector', gr_image)
    cv2.resizeWindow('button_detector', 320, 240)
    
    if circles_green is not None:
        return green_found
    
    if circles_red is not None:
        return red_found
   

    
        

