import cv2
import numpy as np

def perform_stitch( image1, image2):
    
    # Initialize the SIFT feature detector and extractor

    sift = cv2.SIFT_create()


    # Detect keypoints and compute descriptors for both images

    keypoints1, descriptors1 = sift.detectAndCompute(image1, None)

    keypoints2, descriptors2 = sift.detectAndCompute(image2, None)        
    
    # Draw keypoints on the images

    image1_keypoints = cv2.drawKeypoints(image1, keypoints1, None)

    image2_keypoints = cv2.drawKeypoints(image2, keypoints2, None)


    # Display the images with keypoints

    cv2.namedWindow("Image 1 with Keypoints",cv2.WINDOW_NORMAL)

    cv2.namedWindow("Image 2 with Keypoints",cv2.WINDOW_NORMAL)

    cv2.imshow("Image 1 with Keypoints", image1_keypoints)

    cv2.imshow("Image 2 with Keypoints", image2_keypoints)

    cv2.resizeWindow("Image 1 with Keypoints",320,240)

    cv2.resizeWindow("Image 2 with Keypoints",320,240) 
    
    
    #=====================================================================================
    #FLANN matcher
    flann_matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
    knn_matches = flann_matcher.knnMatch(descriptors1, descriptors2, 2)
    
    #filter matches with lowe's ratio test
    ratio_thresh = 0.7
    good_matches = []
    for m,n in knn_matches:
        if m.distance < ratio_thresh * n.distance:
            good_matches.append(m)
    
    num_matches = 50
    
    #-- Draw matches
    img_matches = np.empty((max(image1.shape[0], image2.shape[0]), image1.shape[1]+image2.shape[1], 3), dtype=np.uint8)
    image_with_matches = cv2.drawMatches(image1, keypoints1, image2, keypoints2, good_matches[:num_matches], img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    
    cv2.namedWindow("FLANN Matching", cv2.WINDOW_NORMAL)

    cv2.imshow('FLANN Matching', image_with_matches)

    cv2.resizeWindow('FLANN Matching',640,480)
    #=====================================================================================               
    
    # BRUTE FORCE METHOD
    #=======================================================================================
    # # Initialize the feature matcher using brute-force matching

    # bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    # print("Started brute force")


    # # Match the descriptors using brute-force matching

    # matches_bf = bf.match(descriptors1, descriptors2)
    # print("Matched brute force")


    # # Sort the matches by distance (lower is better)

    # matches_bf = sorted(matches_bf, key=lambda x: x.distance)
    # print("Sorted brute force")

    # # Draw the top N matches

    # num_matches = 50

    # image_matches_bf = cv2.drawMatches(image1, keypoints1, image2, keypoints2, matches_bf[:num_matches], None)
    # print("Match drown")

    # Display the images with matches
    # print("reached brute force")
    # cv2.namedWindow("Brute-Force Matching", cv2.WINDOW_NORMAL)

    # cv2.imshow('Brute-Force Matching', image_matches_bf)

    # cv2.resizeWindow('Brute-Force Matching',640,480)
    # print("passed brute force")
    
    #===============================================================================
    
    # Estimate the homography matrix using RANSAC


    # First we need to convert the matched keypoints into a format that that can be used by findHomography 

    tp=[] # target points

    qp=[] # query points
    

    for matches in knn_matches:
        for m in matches:

            tp.append(keypoints2[m.trainIdx].pt)

            qp.append(keypoints1[m.queryIdx].pt)

    tp,qp=np.float32((tp,qp))
    
    


    homography, _ = cv2.findHomography(tp, qp, cv2.RANSAC, 5.0)
    "post homography. If doesn't work swap qp with tp"


    # Print the estimated homography matrix

    "Estimated Homography Matrix:"

    print(homography)


    # Warp the second image using the homography

    # We make the resulting image slightly wider to accommodate the new rotated image

    result = cv2.warpPerspective(image2, homography, (image1.shape[1]+300, image1.shape[0]))


    # Blending the warped image with the first image using alpha blending

    # First create a new image large enough to accommodate the stitched images

    padded_left_img = cv2.copyMakeBorder(image1, 0, 0, 0, result.shape[1] - image1.shape[1],cv2.BORDER_CONSTANT )

    alpha = 0.5  # blending factor

    blended_image = cv2.addWeighted(padded_left_img, alpha, result, 1 - alpha, 0)


    # Display the blended image

    cv2.namedWindow("Blended Image", cv2.WINDOW_NORMAL)

    cv2.imshow("Blended Image",blended_image)

    cv2.resizeWindow('Blended Image',1280,960)
            
    return