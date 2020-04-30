#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import moveit_commander
import geometry_msgs.msg

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import roslib




def talker():
    print('Publishing marker')
    rospy.init_node('talker',anonymous=True)
    marker = Marker()
    marker.header.frame_id = "panda_link0"
    marker.type = marker.CUBE
    #marker.type = marker.MESH_RESOURCE;
    #path = roslib.packages.get_pkg_dir("moveit_tutorials") + "/table.STL"
    #print(path)
    #marker.mesh_resource = 'package://' + 'moveit_tutorials' + '/' + 'table.stl'
    marker.action = marker.ADD
    box_height = 0.4
    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = box_height
    marker.color.a = 1.0
    marker.color.r = 0.750
    marker.color.g = 0.20
    marker.color.b = 0.20
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0.65
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.id = 1

    table = Marker()
    table.header.frame_id = "panda_link0"
    #marker.type = marker.CUBE
    table.type = marker.MESH_RESOURCE;
    #path = roslib.packages.get_pkg_dir("moveit_tutorials") + "/table.STL"
    #print(path)
    table.mesh_resource = 'package://' + 'moveit_tutorials' + '/' + 'table.stl'
    table.action = marker.ADD
    table.scale.x = 0.0175
    table.scale.y = 0.0175
    table.scale.z = 0.0175
    table.color.a = 1.0
    table.color.r = 0.0
    table.color.g = 0.0
    table.color.b = 0.2
    table.pose.orientation.w = 1.0
    table.pose.orientation.x = 1.0
    table.pose.orientation.y = 0.0
    table.pose.orientation.z = 0.0
    table.pose.position.x = 1.25
    table.pose.position.y = 0
    table.pose.position.z = 0
    table.id = 2

    patient = Marker()
    patient.header.frame_id = "panda_link0"
    #marker.type = marker.CUBE
    patient.type = marker.MESH_RESOURCE;
    #path = roslib.packages.get_pkg_dir("moveit_tutorials") + "/table.STL"
    #print(path)
    patient.mesh_resource = 'package://' + 'moveit_tutorials' + '/' + 'patient.dae'
    patient.action = marker.ADD
    patient.scale.x = 0.4
    patient.scale.y = 0.4
    patient.scale.z = 0.4
    patient.color.a = 1.0
    patient.color.r = 1.0
    patient.color.g = 1.0
    patient.color.b = 1.0
    patient.pose.orientation.w = 0.5
    patient.pose.orientation.x = -0.5
    patient.pose.orientation.y = -0.5
    patient.pose.orientation.z = 0.5
    patient.pose.position.x = 1.65
    patient.pose.position.y = 0
    patient.pose.position.z = 0.1
    patient.id = 3



    markerArray = MarkerArray()
    markerArray.markers.append(marker)
    #markerArray.markers.append(table)
    #markerArray.markers.append(patient)


    topic = 'visualization_marker_array'
    pub = rospy.Publisher(topic, MarkerArray, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(markerArray)
        #scene_pub.publish(PlanningScene)
        rate.sleep()


if __name__ == '__main__':
    try:


        talker()
    except rospy.ROSInterruptException:
        pass
