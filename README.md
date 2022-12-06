# Cockroach Killing Robot
![cockroach picking](bugshort.gif)

### Team members
Christopher Xu, Vanessa Blas, Ethan Hsiao

### Task
Our robot arm identifies moving cockroaches on table, terminate the roaches, then dispose of the roaches in a trash bin. There is a flyswatter and a gripper at the end effector. It uses a camera above the table to detect cockroaches, and if the cockroach stops for a moment the robot rotates the swatter down to stun the roach. The arm will then pick up the roach and drop it in the trash to the side. 

In the simulation, the cockroach is programmed to jump around randomly on the table while staying within the workspace of the robot arm. Every once in a while, the roach will stop for a few seconds. 

### Requirements
Computer vision and ROS scripts are written in Python 3, with the pip libraries numpy, pandas, and pyquaternion. Drivers are imported in C++ for a UR5 robot. Comfirmed to work on ROS Noetic installed on Ubuntu 20.04.

### To run

Build the workspace: 
```
cd catkin_ws
catkin_make
```

Before each of the following commands:
```
source devel/setup.bash
```

Launch the world in one terminal:
```
roslaunch levelManager lego_world.launch
```

Spawn a moving roach in the world in another terminal:
```
rosrun levelManager levelManager.py m
```

Start the kinematics process in a third terminal:
```
rosrun motion_planning motion_planning.py
```

### Motivation
To find a modern solution to our century old problem. We need something that can kill cockroaches without human assistance so humans can spend time on more important tasks. 

### Challenges
- Cockroach movement
  - implemented with a gazebo service to apply a wrench in a random direction at random intervals
  - cockroach movement is jerky
  - on rare occasions the cockroach falls through the table because of a collision bug 
- Inaccurate position detection 
  - grasp location is more inaccurate farther away from the center of the camera frame because of focal length and distortion
  - detected position is based on the centroid of a color mask blob detection, so the robot does not account for the cockroach's rotation
- Slipping 
  - the cockroach slips out from the gripper because of unpredictable forces from collision and intersection
  - increased friction on the cockroach surface did not help
  - eventually solved by creating a dynamic link in simulation between the gripper and the cockroach while the gripper is closed, but this often crashes the robot controller and is slow to release 

### Next steps
- Handle multiple cockroaches at once. Instead of dealing with each cockroach in turn, hit all cockroaches in vision and then throw them away. 
- Currently the vision system labels all orange objects bigger than a threshold a cockroach. In the future, only kill if it has a cockroach shape.
- Avoid obstacles on the table with more advanced motion planning
- Tape a flyswatter on the real robot arm and test this code in the real world 

### References
- UR5 robot arm and kinect camera in Gazebo simulation: https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation 
- XML documentation: http://wiki.ros.org/urdf/XML 
- Lego Minifig Model: https://grabcad.com/library/lego-minifig-man-woman-figure-1 
- Cockroach 3D model: https://www.thingiverse.com/thing:1241800 

Note: the robot is named Ethan’s Mom, inspired by Ethan’s mom’s extraordinary cockroach killing skills
