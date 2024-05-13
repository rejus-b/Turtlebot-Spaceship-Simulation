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

def detect_window(image):
    """
    Function for detecting windows from the camera view
    input: image already converted using self.bridge.imgmsg_to_cv2(data, "bgr8")
    output: True or False. 
    The output might change as we will probably need to either pin the position of the window on the map
    or use the coordinate on the image to rotate the robot
    """
    # convert image in HSV
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # apply gaussian blur
    blurred_image = cv2.GaussianBlur(grayscale, (5,5), 0)
    
    # Enanche contrast
    equalized_image = cv2.equalizeHist(blurred_image)
    
    # thresholding
    # _, binary_mask = cv2.threshold(equalized_image, 200, 255, cv2.THRESH_BINARY)
        
    # FIlter everything but dark colours
    masked_dark = cv2.inRange(equalized_image, 0, 40)
    masked_white = cv2.inRange(equalized_image, 200, 255)
    
    # find contours       
    contours_dark, _= cv2.findContours(masked_dark, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    contours_white, _ = cv2.findContours(masked_white, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    
    
    # loop over the contours
    
    if len(contours_dark) > 0:
        
        # find biggest contour 
        max_dark_contour = max(contours_dark, key=cv2.contourArea, default=None)
        if max_dark_contour is not None:
            window_detected = True
            print("Window detected")
            
            #compute the bounding rectangle
            for contour in contours_dark:
                
                if cv2.contourArea(contour) > 1000:
                    peri = cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
                    
                    if len(approx) == 4:
                        x,y,w,h = cv2.boundingRect(contour)
                        ratio = float(w)/h
                        if ratio <= 0.9 or ratio >= 1.1:
                            image = cv2.drawContours(image, contour, -1, (0, 0, 255), 2)
                            image = cv2.rectangle(image, (x,y), (x+w, y+h), (0,255,0), 2)
    
    if len(contours_white) > 0:
        max_dark_contour = max(contours_dark, key=cv2.contourArea, default=None)


    
    
    
    # Camera feed
    cv2.namedWindow('dark_mask',cv2.WINDOW_NORMAL) 
    cv2.imshow('dark_mask', masked_dark)
    cv2.resizeWindow('dark_mask', 320, 240) 
    
        # Camera feed
    cv2.namedWindow('white_mask',cv2.WINDOW_NORMAL) 
    cv2.imshow('white_mask', masked_white)
    cv2.resizeWindow('white_mask', 320, 240) 
    cv2.waitKey(1)
    
    return None