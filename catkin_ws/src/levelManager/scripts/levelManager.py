#!/usr/bin/python3
from pandas import array
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler
import rospy, rospkg, rosservice
import sys
import time
import random
import numpy as np

import xml.etree.ElementTree as ET

path = rospkg.RosPack().get_path("levelManager")

costruzioni = ['costruzione-1', 'costruzione-2']

def randomCostruzione():
	return random.choice(costruzioni)

def getPose(modelEl):
	strpose = modelEl.find('pose').text
	return [float(x) for x in strpose.split(" ")]

def get_Name_Type(modelEl):
	if modelEl.tag == 'model':
		name = modelEl.attrib['name']
	else:
		name = modelEl.find('name').text
	return name, name.split('_')[0]
	

def get_Parent_Child(jointEl):
	parent = jointEl.find('parent').text.split('::')[0]
	child = jointEl.find('child').text.split('::')[0]
	return parent, child


def getLego4Costruzione(select=None):
	nome_cost = randomCostruzione()
	if select is not None: nome_cost = costruzioni[select]
	print("spawning", nome_cost)

	tree = ET.parse(f'{path}/lego_models/{nome_cost}/model.sdf')
	root = tree.getroot()
	costruzioneEl = root.find('model')

	brickEls = []
	for modEl in costruzioneEl:
		if modEl.tag in ['model', 'include']:
			brickEls.append(modEl)

	models = ModelStates()
	for bEl in brickEls:
		pose = getPose(bEl)
		models.name.append(get_Name_Type(bEl)[1])
		rot = Quaternion(*quaternion_from_euler(*pose[3:]))
		models.pose.append(Pose(Point(*pose[:3]), rot))

	rospy.init_node("levelManager")
	istruzioni = rospy.Publisher("costruzioneIstruzioni", ModelStates, queue_size=1)
	istruzioni.publish(models)

	return models


def changeModelColor(model_xml, color):
	root = ET.XML(model_xml)
	root.find('.//material/script/name').text = color
	return ET.tostring(root, encoding='unicode')	


#DEFAULT PARAMETERS
package_name = "levelManager"



#get model path
def getModelPath(model):
	pkgPath = rospkg.RosPack().get_path(package_name)
	return f'{pkgPath}/lego_models/{model}/model.sdf'


#functiont to spawn model
def spawn_model(model, pos, name=None, ref_frame='world', color=None):
	
	if name is None:
		name = model
	
	model_xml = open(getModelPath(model), 'r').read()
	if color is not None:
		model_xml = changeModelColor(model_xml, color)

	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	return spawn_model_client(model_name=name, 
	    model_xml=model_xml,
	    robot_namespace='/foo',
	    initial_pose=pos,
	    reference_frame=ref_frame)


if __name__ == '__main__':

	try:
		if '/gazebo/spawn_sdf_model' not in rosservice.get_service_list():
			print("Waining gazebo service..")
			rospy.wait_for_service('/gazebo/spawn_sdf_model')
		
		
		arg = sys.argv[1]

		if arg in ['roach', 'add', 'spawn', 'r']:
			delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
			delete_model_client(model_name='cockroach')

			rot = Quaternion(*quaternion_from_euler(0, 0, 0))
			point = Point(0, -0.5, 1)
			pose =  Pose(point, rot)
			spawn_model(model='cockroach', pos=pose, color='Gazebo/Orange')
			print(f"Added a roach")
		elif arg in ['unroach', 'del', 'despawn', 'd']:
			delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
			print(delete_model_client(model_name='cockroach'))
		else:
			print("Usage: rosrun levelManager levelManager.py roach|unroach")
			exit()

	except rosservice.ROSServiceIOException as err:
		print("No ROS master execution")
		pass
	except rospy.ROSInterruptException as err:
		print(err)
		pass
	except rospy.service.ServiceException:
		print("No Gazebo services in execution")
		pass
	except rospkg.common.ResourceNotFound:
		print(f"Package not found: '{package_name}'")
		pass
	except FileNotFoundError as err:
		print(f"Model not found: \n{err}")
		print(f"Check model in folder'{package_name}/lego_models'")
		pass
