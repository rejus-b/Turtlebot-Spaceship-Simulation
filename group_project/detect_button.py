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

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
    green_image = cv2.bitwise_and(image, image, mask=green_mask)
    red_image = cv2.bitwise_and(image, image, mask=red_mask)

        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        #grey_green = cv2.cvtColor(green_image, cv2.COLOR_BGR2GRAY)
        #grey_red = cv2.cvtColor(red_image, cv2.COLOR_BGR2GRAY)
    contours_green, hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c_green = 0
    c_red = 0
    
    gr_image = cv2.bitwise_or(green_image,red_image)
    
    cv2.namedWindow('button_detector',cv2.WINDOW_NORMAL) 
    cv2.imshow('button_detector', gr_image)
    cv2.resizeWindow('button_detector', 320, 240)
    
    # Loop over the contours
    if (len(contours_green)>0 or len(contours_red)>0):
            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            
        c_green = max(contours_green, key=cv2.contourArea, default=None)
        c_red = max(contours_red, key=cv2.contourArea, default=None)


            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
        if (c_green is not None and cv2.contourArea(c_green) > 20): #<What do you think is a suitable area?>
                # Alter the value of the flag
            return green_found

            
        if ( c_red is not None and cv2.contourArea(c_red) > 20):
            return red_found
        

