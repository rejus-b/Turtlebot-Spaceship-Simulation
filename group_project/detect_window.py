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
    """
    Returns a tuple of window detected flag and the rotation_indication
    rotation indication can be:
    0 if no window is detected
    1 if the center of the camera is looking at the frame
    -10 if the camera is shifted left to the frame
    10 if the camera is shifted right to the frame 
    """
    
    # convert image in HSV
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # apply gaussian blur
    blurred_image = cv2.GaussianBlur(grayscale, (5,5), 0)
    
    rotation_command = 0 # when no potential windows are detected
    window_detected = False
    # FIlter everything but dark colours
    masked_dark = cv2.inRange(blurred_image, 0, 40)
    masked_white = cv2.inRange(blurred_image, 200, 255)
    
    # find contours       
    contours_dark, _= cv2.findContours(masked_dark, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    contours_white, _ = cv2.findContours(masked_white, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    
    
    
    
    # loop over the contours
    
    if len(contours_dark) > 0:
        
        # find biggest contour 
        max_dark_contour = max(contours_dark, key=cv2.contourArea, default=None)
        if max_dark_contour is not None:
                
            window_detected = True
        
            # filter all the contours less than 1000
            contours_dark = [contour for contour in contours_dark if cv2.contourArea(contour) >= 1000]
            
            #compute the bounding rectangle
            for contour_dark in contours_dark:
                
                
                
                if cv2.contourArea(contour_dark) > 1000:
                    
                    peri_dark = cv2.arcLength(contour_dark, True)
                    approx_dark = cv2.approxPolyDP(contour_dark, 0.02 * peri_dark, True)
                    
                    
                    if len(approx_dark) == 4:
                        x_d,y_d,w_d,h_d = cv2.boundingRect(contour_dark)
                        
                        # Define a region of interest (ROI) within the bounding rectangle
                        roi = grayscale[y_d:y_d+h_d, x_d:x_d+w_d]
                                            
                        ratio = float(w_d)/h_d
                        if ratio >=0.90  and ratio <= 1.5:
                            #image = cv2.drawContours(image, contour_dark, -1, (0, 0, 255), 2) #draw red contours
                            #image = cv2.rectangle(image, (x_d,y_d), (x_d+w_d, y_d+h_d), (0,255,0), 2) # draw green rectangle
                            
                            
                            # detect if the rect is in the center
                            
                            image_center_x = image.shape[1] // 2
                            window_center_x = x_d + w_d // 2
                            
                            if window_center_x < image_center_x - 15:
                                rotation_command = 10  # Rotate left
                                
                            elif window_center_x > image_center_x + 15:
                                rotation_command = -10  # Rotate right
                            else:
                                rotation_command = 1 # Good position
                                    
        
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
    
    return (window_detected, rotation_command)