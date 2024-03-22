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
    
    Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    #lower and upper bounds for dark colors
    hsv_dark_lower = np.array([0, 0, 0]) 
    hsv_dark_upper = np.array([180, 50, 30])
    
    # FIlter everything but dark colours
    masked_dark = cv2.inRange(Hsv_image, hsv_dark_lower, hsv_dark_upper)
    
    # Apply the mask to the original image using the cv2.bitwise_and() method
        
    
    
    
    # Camera feed
    cv2.namedWindow('window_detector',cv2.WINDOW_NORMAL) 
    cv2.imshow('window_detector', masked_dark)
    cv2.resizeWindow('window_detector', 320, 240) 
    
    return None