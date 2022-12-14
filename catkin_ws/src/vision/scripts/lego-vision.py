#! /usr/bin/env python3

import cv2 as cv2
import cv2 as cv
import numpy as np
import message_filters
import rospy
import sys
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rospkg import RosPack # get abs path
from os import path # get home path
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *

# Global variables
path_yolo = path.join(path.expanduser('~'), 'yolov5')
path_vision = RosPack().get_path('vision')
path_weigths = path.join(path_vision, 'weigths')

cam_point = (-0.44, -0.5, 1.58)
height_tavolo = 0.74
dist_tavolo = None
origin = None
model = None
model_orientation = None


argv = sys.argv
a_show = '-show' in argv

# Utility Functions

def get_dist_tavolo(depth, hsv, img_draw):
    global dist_tavolo

    #color = (120,1,190)
    #mask = get_lego_mask(color, hsv, (5, 5, 5))
    #dist_tavolo = depth[mask].max()
    #if dist_tavolo > 1: dist_tavolo -= height_tavolo
    dist_tavolo = np.nanmax(depth)

def get_origin(img):
    global origin
    origin = np.array(img.shape[1::-1]) // 2

def get_lego_distance(depth):
    return depth.min()

def get_lego_color(center, rgb):
    return rgb[center].tolist()

def get_lego_mask(color, hsv, toll = (20, 20, 255)):
    thresh = np.array(color)
    mintoll = thresh - np.array([toll[0], toll[1], min(thresh[2]-1, toll[2])])
    maxtoll = thresh + np.array(toll)
    return cv.inRange(hsv, mintoll, maxtoll)

def getDepthAxis(height, lego):
    X, Y, Z = (int(x) for x in lego[1:8:3])
    #Z = (0.038, 0.057) X = (0.031, 0.063) Y = (0.031, 0.063, 0.095, 0.127)
    rapZ = height / 0.019 - 1
    pinZ = round(rapZ)
    rapXY = height / 0.032
    pinXY = round(rapXY)
    errZ = abs(pinZ - rapZ) + max(pinZ - 2, 0)
    errXY = abs(pinXY - rapXY) + max(pinXY - 4, 0)
    
    if errZ < errXY:
        return pinZ, 2, pinZ == Z    # pin, is ax Z, match
    else:
        if pinXY == Y: return pinXY, 1, True
        else: return pinXY, 0, pinXY == X

def point_distorption(point, height, origin):
    p = dist_tavolo / (dist_tavolo - height)
    point = point - origin
    return p * point + origin

def point_inverse_distortption(point, height):
    p = dist_tavolo / (dist_tavolo - height)
    point = point - origin
    return point / p + origin

def myimshow(title, img):
    def mouseCB(event,x,y,a,b):
        print(x, y, img[y, x], "\r",end='',flush=True)
        print("\033[K", end='')
    cv.imshow(title, img)
    cv.setMouseCallback(title, mouseCB)
    cv.waitKey()

# ----------------- LOCALIZATION ----------------- #

#image processing
def process_image(rgb, depth):    
    
    img_draw = rgb.copy()
    hsv = cv.cvtColor(rgb, cv.COLOR_BGR2HSV)

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
        if(keypoints[i].size > 15):
            blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
    num_blobs = len(blob_image_center)

    roachX = (blob_image_center[0][0] - 317)*(0.25/-145)
    roachY = (blob_image_center[0][1] - 235.8)*(0.25/145) - 0.5

    imgx = (int)(blob_image_center[0][0])
    imgy = (int)(blob_image_center[0][1])

    print(roachX, roachY)

    res = ModelStates()
    res.name.append('roach')
    res.pose.append(Pose(Point(x=roachX, y=roachY, z=0), Quaternion()))

    pub.publish(res)

    

    # cv.imshow("mask", mask_image)
    

    # if a_show:
    img_draw = cv2.circle(img_draw, (imgx, imgy) , 20, color=(255, 0, 0), thickness=3)
    cv.imshow("vision-results.png", img_draw)
    cv.waitKey()


def process_CB(image_rgb, image_depth):
    t_start = time.time()
    #from standard message image to opencv image
    rgb = CvBridge().imgmsg_to_cv2(image_rgb, "bgr8")                                                
    depth = CvBridge().imgmsg_to_cv2(image_depth, "32FC1")
    
    process_image(rgb, depth)

    print("Time:", time.time() - t_start)
    rospy.signal_shutdown(0)


#init node function
def start_node():
    global pub

    print("Starting Node Vision 1.0")

    rospy.init_node('vision') 
    
    print("Subscribing to camera images")
    #topics subscription
    rgb = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth = message_filters.Subscriber("/camera/depth/image_raw", Image)
    
    #publisher results
    pub=rospy.Publisher("lego_detections", ModelStates, queue_size=1)

    print("Localization is starting.. ")
    print("(Waiting for images..)", end='\r'), print(end='\033[K')
    
    #images synchronization
    syncro = message_filters.TimeSynchronizer([rgb, depth], 1, reset=True)
    syncro.registerCallback(process_CB)
    
    #keep node always alive
    rospy.spin() 
    pass



if __name__ == '__main__':

    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
