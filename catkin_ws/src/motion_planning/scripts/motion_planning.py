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


import message_filters
import sys
import time

from sensor_msgs.msg import Image
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

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
    # if gazebo_model_name is not None:
    #     req = AttachRequest()
    #     req.model_name_1 = gazebo_model_name
    #     req.link_name_1 = "link"
    #     req.model_name_2 = "robot"
    #     req.link_name_2 = "wrist_3_link"
    #     attach_srv.call(req)


def open_gripper(gazebo_model_name=None):
    set_gripper(0.0)

    # Destroy dynamic joint
    # if gazebo_model_name is not None:
    #     req = AttachRequest()
    #     req.model_name_1 = gazebo_model_name
    #     req.link_name_1 = "link"
    #     req.model_name_2 = "robot"
    #     req.link_name_2 = "wrist_3_link"
    #     detach_srv.call(req)


def set_model_fixed(model_name):
    req = AttachRequest()
    req.model_name_1 = model_name
    req.link_name_1 = "link"
    req.model_name_2 = "ground_plane"
    req.link_name_2 = "link"
    attach_srv.call(req)

    req = SetStaticRequest()
    print("{} TO HOME".format(model_name))
    req.model_name = model_name
    req.link_name = "link"
    req.set_static = True

    setstatic_srv.call(req)


def set_gripper(value):
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value  # From 0.0 to 0.8
    goal.command.max_effort = 1  # # Do not limit the effort
    action_gripper.send_goal_and_wait(goal, rospy.Duration(10))

    return action_gripper.get_result()


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

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)

    open_gripper()


    image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)

    rgb = np.zeros((100,100,3), dtype=np.uint8)
    annotated = np.zeros((100,100,3), dtype=np.uint8)
    locs = np.zeros((10, 2))

    def image_callback(imagemsg):
        global rgb
        global annotated
        rgb = CvBridge().imgmsg_to_cv2(imagemsg, "bgr8")
        
        loc = vision.get_center(rgb)
        np.roll(locs, -1)
        locs[-1] = loc
        
        if(np.linalg.norm(loc[0] - loc) < 0.1):
            print('not moved')


    image_sub.registerCallback(image_callback)
    
    print("start loop")

    while(True):    

        cv2.imshow('rgb', rgb)
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