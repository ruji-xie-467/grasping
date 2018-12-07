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
from tf.transformations import quaternion_from_matrix, quaternion_matrix



if __name__ == '__main__':
    rospy.init_node('dummy_point')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10) 
    mat = quaternion_matrix([0,0,0,1])
    mat[0:3, 2] = [0,0,-1]
    mat[0:3, 1] = [0,-1,0]
    # mat[0:3, 0] = []
    q = quaternion_from_matrix(mat)

    while not rospy.is_shutdown():
        br.sendTransform([0.7,0.2,0],
                        q,
                        rospy.Time.now(),
                        "sample_pregrasp_0",
                        "base")
        br.sendTransform([0.7,0.2,-0.2],
                        q,
                        rospy.Time.now(),
                        "sample_grasp_0",
                        "base")
        rate.sleep()

