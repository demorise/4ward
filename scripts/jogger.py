#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Joy
import roslib


print "============ Press Begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('jogger',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)

planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

eef_link = group.get_end_effector_link()
print "============ End effector: %s" % eef_link
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""

scene = moveit_commander.PlanningSceneInterface()
box_name = 'box'



def add_box():

  #box_name box_name
  #scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
  box_pose = geometry_msgs.msg.PoseStamped()
  box_pose.header.frame_id = "panda_link0"
  box_pose.pose.orientation.w = 1.0
  box_pose.pose.orientation.x = 0.0
  box_pose.pose.orientation.z = 0.0
  box_pose.pose.orientation.y = 0.0
  box_pose.pose.position.x = 0.65#1.25
  box_pose.pose.position.y = 0
  box_pose.pose.position.z = 0#-0.08
  box_name = "box"
  print "============ Press `Enter` to add a box to the planning scene ..."
  raw_input()
  scene.add_box(box_name, box_pose, size=(0.4, 0.4, 0.4))#size=(3, 1, 0.1))
  #path = roslib.packages.get_pkg_dir("moveit_tutorials") + "/table.stl"
  #print(path)
  #scene.add_mesh(box_name,box_pose,path,  size=(0.0175, 0.0175, 0.0175))
   ## END_SUB_TUTORIAL
  # Copy local variables back to class variables. In practice, you should use the class
  # variables directly unless you have a good reason not to.
  #self.box_name=box_name



def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True




def go_to_joint_state(joint_pose, group):
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = joint_pose[0]
  joint_goal[1] = joint_pose[1]
  joint_goal[2] = joint_pose[2]
  joint_goal[3] = joint_pose[3]
  joint_goal[4] = joint_pose[4]
  joint_goal[5] = joint_pose[5]
  joint_goal[6] = joint_pose[6]
  #print('********',joint_goal)

  group.go(joint_goal, wait=True)
  group.stop()
  current_joints = group.get_current_joint_values()
  return all_close(joint_goal, current_joints, 0.01)




def go_to_pose_goal(pose_in, group):
  group.set_pose_target(pose_in)
  plan = group.go(wait=True)
  group.stop()
  group.clear_pose_targets()
  current_pose=group.get_current_pose().pose
  return all_close(pose_in, current_pose, 0.01)




def callback(data):
  #print('Frame: ', data.header.seq)
  #print('Axes')
  #print(data.axes)
  #print('Buttons')
  #print(data.buttons)
  curr_cart_pose = group.get_current_pose().pose
  pose_goal = group.get_current_pose().pose

  curr_joint_state = group.get_current_joint_values()
  joint_goal = group.get_current_joint_values()

  pose_increment = 0.075
  joint_increment = 0.1
  

  if data.buttons[6]==0 and data.buttons[7]==0:
    #jog in X axis
    if data.axes[6]==1:
      pose_goal.position.x = curr_cart_pose.position.x - pose_increment 
    if data.axes[6]==-1:
      pose_goal.position.x = curr_cart_pose.position.x + pose_increment 
    #jog in Y axis
    if data.axes[7]==-1:
      pose_goal.position.y = curr_cart_pose.position.y - pose_increment 
    if data.axes[7]==1:
      pose_goal.position.y = curr_cart_pose.position.y + pose_increment       
    #jog in Z axis
    if data.buttons[0]==1:
      pose_goal.position.z = curr_cart_pose.position.z - pose_increment 
    if data.buttons[4]==1:
      pose_goal.position.z = curr_cart_pose.position.z + pose_increment 
    go_to_pose_goal(pose_goal, group)


  if data.buttons[6]==1:
    if data.buttons[0]==1:
      joint_goal[0]=curr_joint_state[0]-joint_increment
    if data.buttons[1]==1:
      joint_goal[1]=curr_joint_state[1]-joint_increment
    if data.buttons[3]==1:
      joint_goal[2]=curr_joint_state[2]-joint_increment
    if data.buttons[4]==1:
      joint_goal[3]=curr_joint_state[3]-joint_increment
    if data.axes[6]==1:
      joint_goal[4]=curr_joint_state[4]-joint_increment
    if data.axes[6]==-1:
      joint_goal[5]=curr_joint_state[5]-joint_increment
    if data.axes[7]==1 or data.axes[7]==-1:
      joint_goal[6]=curr_joint_state[6]-joint_increment
    go_to_joint_state(joint_goal, group)
     
  if data.buttons[7]==1:
    if data.buttons[0]==1:
      joint_goal[0]=curr_joint_state[0]+joint_increment
    if data.buttons[1]==1:
      joint_goal[1]=curr_joint_state[1]+joint_increment
    if data.buttons[3]==1:
      joint_goal[2]=curr_joint_state[2]+joint_increment
    if data.buttons[4]==1:
      joint_goal[3]=curr_joint_state[3]+joint_increment
    if data.axes[6]==1:
      joint_goal[4]=curr_joint_state[4]+joint_increment
    if data.axes[6]==-1:
      joint_goal[5]=curr_joint_state[5]+joint_increment
    if data.axes[7]==1 or data.axes[7]==-1:
      joint_goal[6]=curr_joint_state[6]+joint_increment
    go_to_joint_state(joint_goal, group)








  #curr_cart_pose = group.get_current_pose().pose
  #pose_goal = group.get_current_pose().pose
  #pose_goal.position.z = curr_cart_pose.position.z + 0.2
  




def main():


  #print "=======Press ENTER to move robot to joint state goal 0======="
  #raw_input()
  #go_to_joint_state([0,    0,   0,    0,   0,    0,    0], group)

  #print "============ Moving to joint state goal 1..."
  #raw_input()
  #tutorial.go_to_joint_state([0.5413,   -1.1310,    0.3037,   -2.7722,    0.1115,    1.8626,    0.3403])
  #tutorial.go_to_joint_state([0.0000,    1.4404,   -0.0000,    0.8355,   -0.0000,    0.6041,   -0.0000])
  #go_to_joint_state([0.7293,    1.6846,   -0.1203,    0.4525,   -0.0595,    1.4252,    0.3700], group)






  #curr_cart_pose = group.get_current_pose().pose
  #pose_goal = group.get_current_pose().pose
  #pose_goal.position.z = curr_cart_pose.position.z + 0.2
  #go_to_pose_goal(pose_goal, group)
  print('press ENTER to remove box')
  raw_input()
  scene.remove_world_object(box_name)
  add_box()
  rospy.Subscriber("joy", Joy, callback)
  rospy.spin()

if __name__ == '__main__':
  main()

