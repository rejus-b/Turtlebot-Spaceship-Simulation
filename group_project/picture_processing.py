import math
import cv2
from cv_bridge import CvBridge
import os
import numpy as np

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
        
        path = os.path.join(os.getcwd(), "src","group-project-group-5", "group_project", "cw_pictures", image_name + ".png")

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
        done = cv2.imwrite(path, poster_roi)
        return poster_roi
        
    except Exception as e:
        print(f"Error: {e}")
        return None
    

"""
Function to resize the smallest jpg amongst the two

input: 
-filename of the first pic
-filename of the second pic
"""
def resize_png_pictures(name1, name2): 
    
    path_1 = find_image_location(name1 + ".png", os.getcwd()) # find name1 frm current path
    path_2 = find_image_location(name2 + ".png", os.getcwd()) 
    
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
        

def resize_png_pictures_manual(name1, width, height): 
    
    path_1 = find_image_location(name1 + ".png", os.getcwd()) # find name1 frm current path 
    
    # Load the two images
    image1 = cv2.imread(path_1)

    # Resize image1 to match the dimensions of image2
    image1_resized = cv2.resize(image1, (width, height))
    # Save the resized image
    cv2.imwrite(path_1, image1_resized)

"""
Function to translate a jpg file to a cv2

input:
-FIlename of a jpg

output:
-cv2 image
"""
def from_png_to_cv2(filename:str):
    
    path= find_image_location(filename + ".png", os.getcwd()) # find name1 frm current path
    
    # Load the images using absolute paths
    image1 = cv2.imread(path)

    return image1


"""
Function to detect earth and moon and calculate the distances
"""
def calculate_distances_from_panorama(panorama_pic):
    panorama_pic =  cv2.imread("src/group-project-group-5/group_project/cw_pictures/"+panorama_pic)
    height_pic, width_pic, _ = panorama_pic.shape
    scaling_factor = 3

    biggest_circles = detect_2_biggest_circle(panorama_pic)
    
    if biggest_circles:
        biggest_circle_diameters = calculate_cirlcles_diameter(biggest_circles)
        planets_real_diameter = calculate_real_diameter(biggest_circle_diameters)
        
        #distance from Earth
        distance_to_earth = round(scaling_factor*((planets_real_diameter[0] * height_pic) / biggest_circle_diameters[0]))
        
        # Distance from Moon
        distance_to_moon = round(scaling_factor * ((planets_real_diameter[1] * height_pic) / biggest_circle_diameters[1]))
        
        # Distance etween Earth and Moon
        distance_between_earth_and_moon = calculate_distance_between_earth_and_moon(biggest_circles, planets_real_diameter)
        
        return f"Earth: {distance_to_earth} km \nMoon: {distance_to_moon} km\nDistance: {distance_between_earth_and_moon} km"
        
    else:
        return ("error, nothing happened")

        
        
def detect_2_biggest_circle(panorama_pic):
    # Convert the image to grayscale
    gray = cv2.cvtColor(panorama_pic, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                            param1=50, param2=30, minRadius=10, maxRadius=100)

    # If circles are detected
    if circles is not None:
        # Convert the (x, y, r) coordinates to integers
        circles = np.round(circles[0, :]).astype("int")

        # Sort circles based on radius in descending order
        circles = sorted(circles, key=lambda x: x[2], reverse=True)

        # Take the first two circles as the biggest ones
        biggest_circles = circles[:2]
        
        return biggest_circles
    else:
        return None
    
    
def calculate_cirlcles_diameter(circles):
        # Initialize a list to store the diameters of the biggest circles
    biggest_circle_diameters = []
    
        # Loop through each circle
    for circle in circles:
        # Extract the coordinates and radius of the circle
        x, y, radius = circle

        # Calculate the diameter by doubling the radius
        diameter = 2 * radius

        # Append the diameter to the list
        biggest_circle_diameters.append(diameter)
    
    return biggest_circle_diameters

    

def calculate_real_diameter(biggest_circle_diameters):
    
    earth_real_diameter_km = 12742 #km. real diameter of the earth
    earth_pixel_diameter_px = biggest_circle_diameters[0] #pixel diameter of the earth
    moon_pixel_diameter_px = biggest_circle_diameters[1] #pixel diameter of the moon
    
    # Calculate the pixel ratio
    pixel_ratio = moon_pixel_diameter_px / earth_pixel_diameter_px

    # Calculate the real diameter of the Moon
    moon_real_diameter_km = pixel_ratio * earth_real_diameter_km
    
    return [earth_real_diameter_km, moon_real_diameter_km]


def calculate_distance_between_earth_and_moon(biggest_circles, planets_real_diameters):
    earth_mid_x, earth_mid_y, earth_radius = biggest_circles[0]
    moon_mid_x, moon_mid_y, moon_radius = biggest_circles[1]
    
    pixel_between_earth_and_moon=  math.sqrt((moon_mid_x - earth_mid_x)**2 + ( moon_mid_y - earth_mid_y)**2) - (earth_radius - moon_radius)
    
    km_per_pixel = round(planets_real_diameters[0] / (earth_radius * 2)) # km per pixel
    
    scalar_factor = 3
    
    real_distance_between_earth_and_moon = pixel_between_earth_and_moon * km_per_pixel * scalar_factor
    
    return round(real_distance_between_earth_and_moon)