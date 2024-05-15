import cv2
import numpy as np
import os

from group_project.picture_processing import find_image_location


def perform_stitch( image1, image2, stitched_file_name):
    
      # Create a stitcher object
    stitcher = cv2.Stitcher_create()

    # Stitch the images
    status, stitched_image = stitcher.stitch([image1, image2])

    if status == cv2.Stitcher_OK:
        cv2.namedWindow("Blended Image", cv2.WINDOW_NORMAL)

        cv2.imshow("Blended Image",stitched_image)

        cv2.resizeWindow('Blended Image',1280,960)
        
        path = find_image_location(stitched_file_name + ".jpg", os.getcwd())
        
        
        # Save the cropped poster image to a file
        done = cv2.imwrite(path, stitched_image)
            
            
        return stitched_image
    else:
        print("Stitching failed!")
        return None