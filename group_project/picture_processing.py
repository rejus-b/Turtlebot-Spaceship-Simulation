import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import os

# Initialize CvBridge
bridge = CvBridge()



def find_image_location(filename, search_dir):
    # Walk through the directory tree starting from search_dir
    for root, dirs, files in os.walk(search_dir):
        if filename in files:
            # Found the file, return its location
            return os.path.abspath(os.path.join(root, filename))

    # File not found in the directory tree
    print("\n\n FILE NOT FOUND \n\n")
    return None



"""
Function that crops a picture to only extract the content of the window/frame
inputs:
- frame
- image name

output: either the picture in array or a string containing the error
"""
def from_frame_to_image_for_ml(frame, image_name:str):
    try:
        
        # Convert the image to grayscale
        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # apply gaussian blur
        blurred_image = cv2.GaussianBlur(grayscale, (5,5), 0)
        
        # FIlter everything but dark colours
        masked_dark = cv2.inRange(blurred_image, 0, 40)
        
        # find contours       
        contours, _= cv2.findContours(masked_dark, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find the largest contour (the rectangle/window)
        max_contour = max(contours, key=cv2.contourArea, default=None)
        
        
        # Get the bounding rectangle of the largest contour
        x, y, w, h = cv2.boundingRect(max_contour)
        
        # Crop the image to focus only on the content of the poster
        poster_roi = frame[y:y+h, x:x+w]
        
        # Save the cropped poster image to a file
        done = cv2.imwrite(f"src/group-project-group-5/group_project/{image_name}.jpg", poster_roi)
        return poster_roi
        
    except Exception as e:
        return (f"Error: {e}")
    

"""
Function to resize the smallest jpg amongst the two

input: 
-filename of the first pic
-filename of the second pic
"""
def resize_jpg_pictures(name1, name2): 
    
    path_1 = find_image_location(name1 + ".jpg", os.getcwd()) # find name1 frm current path
    path_2 = find_image_location(name2 + ".jpg", os.getcwd()) 
    
    # Load the two images
    image1 = cv2.imread(path_1)
    image2 = cv2.imread(path_2)

    # Get the dimensions of the two images
    height1, width1, _ = image1.shape
    height2, width2, _ = image2.shape

    # Check which image is smaller
    if height1 * width1 < height2 * width2:
        # Resize image1 to match the dimensions of image2
        image1_resized = cv2.resize(image1, (width2, height2))
        # Save the resized image
        cv2.imwrite(path_1, image1_resized)
    else:
        # Resize image2 to match the dimensions of image1
        image2_resized = cv2.resize(image2, (width1, height1))
        # Save the resized image
        cv2.imwrite(path_2, image2_resized)
        

"""
Function to translate a jpg file to a cv2

input:
-FIlename of a jpg

output:
-cv2 image
"""
def from_jpg_to_cv2(filename:str):
    
    path= find_image_location(filename + ".jpg", os.getcwd()) # find name1 frm current path
    
    # Load the images using absolute paths
    image1 = cv2.imread(path)
    
    # Initialize CvBridge
    bridge = CvBridge()

    # Read the JPEG image
    jpg_image = cv2.imread(path)

    # Convert the image to BGR format
    bgr_image = cv2.cvtColor(jpg_image, cv2.COLOR_BGR2RGB)

    # Convert the BGR image to a ROS image message
    ros_image_msg = bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
    
    pic_cv2 = bridge.imgmsg_to_cv2(ros_image_msg, 'bgr8')
    
    return pic_cv2
