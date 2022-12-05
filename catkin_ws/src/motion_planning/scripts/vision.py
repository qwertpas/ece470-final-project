import cv2 as cv2
import numpy as np


def get_center(rgb):
    
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    #orange
    lower = (0,150, 130)     # orange lower
    upper = (25,255,255)   # orange upper

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv, lower, upper)

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)


    for i in range(num_blobs):
        if(keypoints[i].size > 40):
            # print(keypoints[i].size)
            blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
    num_blobs = len(blob_image_center)

    if num_blobs == 0:
        return None, rgb, (None, None)

    # print(blob_image_center)

    roachX = (blob_image_center[0][0] - 317)*(0.25/-145)
    roachY = (blob_image_center[0][1] - 235.8)*(0.25/145) - 0.5

    imgx = (int)(blob_image_center[0][0])
    imgy = (int)(blob_image_center[0][1])

    # print(roachX, roachY)
    
    annotated = cv2.circle(rgb, (imgx, imgy), radius=10, color=(255,0,0,0.5), thickness=2)

    return np.array([roachX, roachY]), annotated, (imgx, imgy)