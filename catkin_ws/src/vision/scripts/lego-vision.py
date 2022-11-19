#! /usr/bin/env python3

import cv2 as cv
import numpy as np
import torch
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
from pyquaternion import Quaternion as PyQuaternion

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

legoClasses = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']

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


def process_item(imgs, item):

    #images
    rgb, hsv, depth, img_draw = imgs
    #obtaining Yolo informations (class, coordinates, center)
    x1, y1, x2, y2, cn, cl, nm = item.values()


    msg = ModelStates()
    msg.name = nm
    
    msg.pose = Pose(Point(*xyz), Quaternion(x=rot.x,y=rot.y,z=rot.z,w=rot.w))
    
    #pub.publish(msg)
    #print(msg)
    return msg


#image processing
def process_image(rgb, depth):    
    
    img_draw = rgb.copy()
    hsv = cv.cvtColor(rgb, cv.COLOR_BGR2HSV)

    get_dist_tavolo(depth, hsv, img_draw)
    get_origin(rgb)

    #results collecting localization

    #print("Model localization: Start...",end='\r')
    model.conf = 0.6
    results = model(rgb)
    pandino = results.pandas().xyxy[0].to_dict(orient="records")
    #print("Model localization: Finish  ")
        
    # ----
    if depth is not None:
        imgs = (rgb, hsv, depth, img_draw)
        results = [process_item(imgs, item) for item in pandino]
    
    # ----

    msg = ModelStates()
    for point in results:
        if point is not None:
            msg.name.append(point.name)
            msg.pose.append(point.pose)
    pub.publish(msg)


    if a_show:
        cv.imshow("vision-results.png", img_draw)
        cv.waitKey()

    pass

def process_CB(image_rgb, image_depth):
    t_start = time.time()
    #from standard message image to opencv image
    rgb = CvBridge().imgmsg_to_cv2(image_rgb, "bgr8")                                                
    depth = CvBridge().imgmsg_to_cv2(image_depth, "32FC1")
    
    process_image(rgb, depth)

    print("Time:", time.time() - t_start)
    rospy.signal_shutdown(0)
    pass

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

def load_models():
    global model, model_orientation
    
    #yolo model and weights classification
    print("Loading model best.pt")
    weight = path.join(path_weigths, 'best.pt')
    model = torch.hub.load(path_yolo,'custom',path=weight, source='local')

    #yolo model and weights orientation
    print("Loading model orientation.pt")
    weight = path.join(path_weigths, 'depth.pt')
    model_orientation = torch.hub.load(path_yolo,'custom',path=weight, source='local')
    pass

if __name__ == '__main__':

    load_models()
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
