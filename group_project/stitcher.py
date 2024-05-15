import cv2
import numpy as np
import os


def perform_stitch( image1, image2, stitched_file_name):
    
      # Create a stitcher object
    stitcher = cv2.Stitcher_create()

    # Stitch the images
    status, stitched_image = stitcher.stitch([image1, image2])

    if status == cv2.Stitcher_OK:
        cv2.namedWindow("Blended Image", cv2.WINDOW_NORMAL)

        cv2.imshow("Blended Image",stitched_image)

        cv2.resizeWindow('Blended Image',1280,960)
        
        try:
        
            path = os.path.join(os.getcwd(), "src","group-project-group-5", "group_project", "cw_pictures", stitched_file_name + ".png")
            
            
            # Save the cropped poster image to a file
            done = cv2.imwrite(path, stitched_image)
            
        except Exception as e:
            return e
            
            
        return stitched_image
    else:
        print("Stitching failed!")
        return None