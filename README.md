# 4Ward - Robot Guidance with MoveIt #

### ME369P  Ademola Oridate, Ed Hu, Doug Feicht

This project explores Virtual Fixtures, a method of assisting or constraining robot motion, using python and ROS/moveIt for visualization. 

## Getting Started
Python code was developed to enable a joystick to be used to simulate robot motion in RViz. 
These instructions should help you install and run the project on your local machine for development and testing purposes. 

### Prerequisites
The code has been developed and tested with Ubuntu 14.04 and ROS Kinetic with MoveIt Kinetic, running on one or more X86-64 PCs using an Xbox One controller. ROS, moveIt and ros key packages should be installed per these instructions

- [ ] ROS Kinetic installation [wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [ ] ROS Network setup [wiki](http://wiki.ros.org/ROS/NetworkSetup)
- [ ] ROS Multiple Machines [tutorial](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
- [ ] Configuring and Using a Linux-Supported Joystick with ROS [wiki](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
- [ ] MoveIt Kinetic installation [wiki](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)
- [ ] Move Group Python Interface [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html) 

### Installing
To install the software, 
- [ ] Clone the repository containing the latest scene objects, URDF robot definition, and python scripts.
- [ ] Unpack the scripts folder. Place files in **_...folder location...._**
- [ ] Open 4 terminal windows and execute these commands in order, each in their own window.

```
roslaunch panda_moveit_config demo.launch            # Launches RViz visualization tool
rosrun joy joy_node                                  # Launches joystick controller
rosrun four_ward add_objects_and_markers.py          # loads workspace objects and markers     
rosrun four_ward jogger.py                           # robot motion code
```
## Using the Xbox One controller
![Xbox One joystick mapping](https://github.com/demorise/four_ward/blob/master/Xbox%20controller%20joystick%20mapping.png?raw=true)

The ROS joy_node obtains key data from the joystick and publishes rostopic 'joy' mesages (above) over the ROS network to a jogger node that converts the button and axes joystick outputs into cartesian and joint move commands. 
Three groups of controller buttons are mapped to robot drive functions as follows:
- D-pad "cross" (lower left): Drives robot parallel to the X and/or Y axis, one increment per key press.
- Right & Left fron "bumpers": Selects joint and sign for joint movement
- "YXBA" buttons: jog individual robot joints one increment based on front bumpers state


## Jogger Code Overview
[Jogger Code Overview](https://github.com/demorise/four_ward/blob/master/Jogger%20Code%20Overview.png?raw=true)

The jogger code receives joystick data and calls a **_moveIt_commander_** method to move the robot within the scene.
First, modules are imported and a planning group instantiated for the Franka Panda robot arm.
```python 
import rospy
import moveit_commander
group = moveit_commander.MoveGroupCommander('panda_arm')
```
A collision object is added to the planning scene (a box representing surgical table), and the rostopic "joy" is subscribed to.
```python
#Add collision object to planning scene
scene = moveit_commander.PlanningSceneInterface()
scene.add_box(box_name, box_pose, size = (3, 1, 0.1))

#Subscribe to joystick ros topic: ‘joy’
rospy.Subscriber(‘joy’, Joy, callback)
```

## Running the tests

### Cartesian Jogging

### Jog Individual Joints

### Collision Avoidance Test

### Virtual Fixture Test

