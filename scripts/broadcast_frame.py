#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import *
import numpy as np

from baxter_interface import Gripper
from utils import count_down_clock

from std_msgs.msg import Header
import tf
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_matrix, quaternion_matrix



""" 
Run 1)  rosrun baxter_tools enable_robot.py -e
Run 2) rosrun baxter_interface joint_trajectory_action_server.py
Run 3) roslaunch baxter_moveit_config baxter_grippers.launch
"""
rospy.init_node('broad_grasp',
                anonymous=True)


tf_listener = tf.TransformListener(rospy.Duration(1))
while not rospy.is_shutdown:

    qs=[]
    ts_grasp=[]
    ts_pre=[]
    for id_num in range(1,20):

        tf_listener.waitForTransform('base',  "grasp_" + str(id_num), rospy.Time(0), rospy.Duration(2.0))
        t, q = tf_listener.lookupTransform("base", "grasp_" + str(id_num), rospy.Time())

        matrix = quaternion_matrix(q)
        new_matrix = np.eye(4)

        new_matrix[:, 0] = matrix[:, 2]
        new_matrix[:, 1] = -matrix[:, 1]
        new_matrix[:, 2] = matrix[:, 0]

        if  sum(new_matrix[:, 2] * np.array([0,0,-1,0])) < 0.75:  # less means more vertical 
            print('give up {0} grasp'.format(id_num))
            continue

        if abs(sum(new_matrix[:, 1] * np.array([0,0,-1,0]))) > 0.25:   # less means not xie
            print('give up {0} grasp'.format(id_num))
            continue
        
        qs.append(q)

        offset_pre = -0.1
        t_pre = t + new_matrix[0:3, 2] * offset_pre
        ts_pre.append(t_pre)

        offset_grasp = -0.05
        t_grasp = t + new_matrix[0:3, 2] * offset_grasp
        ts_grasp.append(t_grasp)

        q = quaternion_from_matrix(new_matrix)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10) 

    print("Find {0} grasps!".format(len(qs)))
    rospy.set_param('good_grasps', len(qs))
    count_down_clock(3)

    if rospy.get_param('get_grasp_from_server'):

        rospy.set_param('get_grasp_from_server', 0)

        while not rospy.is_shutdown():
            for i in range(len(qs)):
                br.sendTransform(ts_pre[i],
                                qs[i],
                                rospy.Time.now(),
                                "sample_pregrasp_"+str(i),
                                "base")

                br.sendTransform(ts_grasp[i],
                                qs[i],
                                rospy.Time.now(),
                                "sample_grasp_"+str(i),
                                "base")
            rate.sleep()





