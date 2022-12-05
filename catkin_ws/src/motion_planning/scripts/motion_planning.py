#!/usr/bin/python3

import os
import math
import copy
import json
import actionlib
import control_msgs.msg
from controller import ArmController
from gazebo_msgs.msg import ModelStates
import rospy
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
import cv2
from cv_bridge import CvBridge
import vision
from threading import Thread


import message_filters
import sys
import time

from sensor_msgs.msg import Image
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse, Detach, DetachRequest

PKG_PATH = os.path.dirname(os.path.abspath(__file__))


# Compensate for the interlocking height
INTERLOCKING_OFFSET = 0.019

SAFE_X = -0.40
SAFE_Y = -0.13
SURFACE_Z = 0.774

# Resting orientation of the end effector
DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
# Resting position of the end effector
DEFAULT_POS = (-0.1, -0.2, 1.2)

DEFAULT_PATH_TOLERANCE = control_msgs.msg.JointTolerance()
DEFAULT_PATH_TOLERANCE.name = "path_tolerance"
DEFAULT_PATH_TOLERANCE.velocity = 10

def get_gazebo_model_name(model_name, vision_model_pose):
    """
        Get the name of the model inside gazebo. It is needed for link attacher plugin.
    """
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    epsilon = 0.05
    for gazebo_model_name, model_pose in zip(models.name, models.pose):
        if model_name not in gazebo_model_name:
            continue
        # Get everything inside a square of side epsilon centered in vision_model_pose
        ds = abs(model_pose.position.x - vision_model_pose.position.x) + abs(model_pose.position.y - vision_model_pose.position.y)
        if ds <= epsilon:
            return gazebo_model_name
    raise ValueError(f"Model {model_name} at position {vision_model_pose.position.x} {vision_model_pose.position.y} was not found!")


def close_gripper(gazebo_model_name, closure=0.55):
    set_gripper(closure)
    rospy.sleep(0.5)
    # Create dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        res = attach_srv.call(req)
        print(res)


def open_gripper(gazebo_model_name=None):
    set_gripper(0.0)

    rospy.sleep(0.5)

    # Destroy dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        res = detach_srv.call(req)
        print(res)


def stun(model_name):
    req = SetStaticRequest()
    print("{} STUNNED".format(model_name))
    req.model_name = model_name
    req.link_name = "link"
    req.set_static = True
    setstatic_srv.call(req)

def revive(model_name):
    req = SetStaticRequest()
    req.model_name = model_name
    req.link_name = "link"
    req.set_static = False
    print(setstatic_srv.call(req))


def set_gripper(value):
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value  # From 0.0 to 0.8
    goal.command.max_effort = 0.1  
    action_gripper.send_goal_and_wait(goal, rospy.Duration(10))

    return action_gripper.get_result()

rgb = np.zeros((100,100,3), dtype=np.uint8)
annotated = np.zeros((100,100,3), dtype=np.uint8)
locs = np.random.random((10,2))
killing = False


def kill(controller, roachX, roachY):
    global killing
    global locs

    killing = True
    x, y, z = (roachX, roachY, 0.77)
    print(f"Moving to {x} {y} {z}")

    controller.move_to(x, y+0.35, 0.9, target_quat=DEFAULT_QUAT * PyQuaternion(axis=[0, 0, 1], angle=-math.pi/2))

    # controller.move_to(target_quat=DEFAULT_QUAT * PyQuaternion(axis=[0, 0, 1], angle=-math.pi/2) * PyQuaternion(axis=[1, 0, 0], angle=-math.pi/4))
    # controller.move_to(target_quat=DEFAULT_QUAT * PyQuaternion(axis=[1, 0, 0], angle=0))


    # controller.move(dz=-0.30)

    controller.move(delta_quat=PyQuaternion(axis=[0, 1, 0], angle=-math.pi/4))
    controller.move(delta_quat=PyQuaternion(axis=[0, 1, 0], angle=math.pi/4))


    # controller.move(dz=0.2)
    open_gripper(gazebo_model_name='cockroach')


    #pick up
    # controller.move(delta_quat=PyQuaternion(axis=[0, 0, 1], angle=math.pi/2))
    time.sleep(0.5)
    controller.move_to(x, y, z+0.25, step=0.1)
    time.sleep(0.5)
    controller.move_to(x, y, z, step=0.1)
    # set_gripper(0.55)
    time.sleep(1)
    close_gripper(gazebo_model_name='cockroach', closure=0.55)
    time.sleep(0.5)

    # controller.move(dz=0.25)
    controller.move_to(x=0.5, y=0, z=z+0.25, step=0.1)

    time.sleep(1)

    open_gripper(gazebo_model_name='cockroach')
    time.sleep(0.5)

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT*PyQuaternion(axis=[0, 0, 1], angle=-math.pi/4))
   
    time.sleep(1)


    locs = np.random.random((10,2))
    killing = False



if __name__ == "__main__":
    print("Initializing node of kinematics")
    rospy.init_node("send_joints")

    controller = ArmController()

    # Create an action client for the gripper
    action_gripper = actionlib.SimpleActionClient(
        "/gripper_controller/gripper_cmd",
        control_msgs.msg.GripperCommandAction
    )
    print("Waiting for action of gripper controller")
    action_gripper.wait_for_server()

    setstatic_srv = rospy.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    setstatic_srv.wait_for_service()
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT*PyQuaternion(axis=[0, 0, 1], angle=-math.pi/4))

    open_gripper(gazebo_model_name='cockroach')



    image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)

    starttime = rospy.get_time()


    def image_callback(imagemsg):
        global rgb
        global annotated
        global locs

        rgb = CvBridge().imgmsg_to_cv2(imagemsg, "bgr8")
        
        loc, annotated, (imgx, imgy) = vision.get_center(rgb)
        if(loc is None):
            # print('not fond')
            return

        locs = np.roll(locs, -1, axis=0)
        locs[-1] = loc
        
        # print(np.linalg.norm(locs[0] - locs[-1]))
        if(np.linalg.norm(locs[0] - locs[-1]) < 0.005 and rospy.get_time() > starttime + 4):
            annotated = cv2.putText(annotated, 'kill!', org=(imgx-10, imgy-20), fontFace=0, fontScale=1, thickness=2, color=(0,0,255))

            if(not killing):  
                print('kill')          
                thread = Thread(target = kill, args = (controller, locs[-1][0], locs[-1][1]))
                thread.start()




    image_sub.registerCallback(image_callback)
    
    print("start loop")



    while(True):    

        # cv2.imshow('rgb', rgb)
        cv2.imshow('annotated', annotated)
        if cv2.waitKey(1)& 0xFF == ord('q'):
            break
            




            
        


    print(vision_res)

    roachX = vision_res.pose[0].position.x+0.02
    roachY = vision_res.pose[0].position.y

        # print("Waiting for vision to start")

    # controller.move(dz=0.15)

    """
        Go to destination
    """
    x, y, z = (roachX, roachY, 0.8) #hardcoded cockroach location for now
    print(f"Moving to {x} {y} {z}")

    controller.move_to(x, y+0.3, target_quat=DEFAULT_QUAT * PyQuaternion(axis=[0, 0, 1], angle=-math.pi/2))

    # controller.move_to(target_quat=DEFAULT_QUAT * PyQuaternion(axis=[0, 0, 1], angle=-math.pi/2) * PyQuaternion(axis=[1, 0, 0], angle=-math.pi/4))
    # controller.move_to(target_quat=DEFAULT_QUAT * PyQuaternion(axis=[1, 0, 0], angle=0))


    controller.move(dz=-0.30)

    controller.move(delta_quat=PyQuaternion(axis=[0, 1, 0], angle=-math.pi/4))
    controller.move(delta_quat=PyQuaternion(axis=[0, 1, 0], angle=math.pi/4))

    controller.move(dz=0.2)



    # Lower the object and release
    controller.move(delta_quat=PyQuaternion(axis=[0, 0, 1], angle=math.pi/2))
    controller.move_to(x, y, z)
    # set_gripper(0.55)
    close_gripper(gazebo_model_name='cockroach', closure=0.55)

    # set_model_fixed(gazebo_model_name)
    controller.move(dz=0.25)
    controller.move_to(x=0.5, y=0)


    open_gripper(gazebo_model_name='cockroach')


    print("Moving to Default Position")
    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    # rospy.sleep(0.4)