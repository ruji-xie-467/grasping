#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np
from gripper import open_gripper, close_gripper
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_matrix, quaternion_matrix

from moveit_msgs.msg import OrientationConstraint
import geometry_msgs

from path_planner import PathPlanner
from baxter_interface import Limb


if __name__ == '__main__':
    rospy.init_node('moveit_node')
   
    planner = PathPlanner("left_arm")
   
    # table
    table_x_width = 1
    table_y_length = 1.4
    table_z_thick = 0.1
    table_position_x = 0.6
    table_position_y = 0.2
    table_position_z = -0.35

    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "base"
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.x = table_position_x
    table_pose.pose.position.y = table_position_y
    table_pose.pose.position.z = table_position_z
    box_name = "table"
    planner.add_box_obstacle(np.array([table_x_width, table_y_length, table_z_thick]), box_name, table_pose) 

    # kinect
    center_of_table_plane = [table_position_x+table_x_width/2,  \
                            table_position_y+table_y_length/2, \
                            table_position_z+table_z_thick/2]
    kinect_pose = geometry_msgs.msg.PoseStamped()
    kinect_pose.header.frame_id = "base"
    kinect_pose.pose.orientation.w = 2.0
    kinect_pose.pose.position.x = table_position_x + 0.55
    kinect_pose.pose.position.y = table_position_y #+ table_y_length/2
    kinect_pose.pose.position.z = 0.4
    box_name = "kinect"
    planner.add_box_obstacle(np.array([0.3,0.3,1]), box_name, kinect_pose) 


    print("Add obstacle successfully")



    # planner.remove_obstacle('table')
    # planner.remove_obstacle('kinect')