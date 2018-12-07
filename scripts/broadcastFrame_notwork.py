#!/usr/bin/env python
import rospy
import moveit_msgs.msg
from geometry_msgs.msg import *
import numpy as np
from utils import count_down_clock

import tf
from tf.transformations import quaternion_from_matrix, quaternion_matrix


if __name__ == '__main__':
    rospy.init_node('broad_grasp',anonymous=True) 
    tf_listener = tf.TransformListener(rospy.Duration(1))
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    ts_pre = []
    ts_grasp = []
    qs= []

    ## Change the axes from approach(R), binormal(G), axis(B) to gripper axes(G,B,R). 

    for id_num in range(1,20):

        tf_listener.waitForTransform('base',  "grasp_" + str(id_num), rospy.Time(0), rospy.Duration(2.0))
        t, q = tf_listener.lookupTransform("base", "grasp_" + str(id_num), rospy.Time())
        matrix = quaternion_matrix(q)
        new_matrix = np.eye(4)
        new_matrix[:, 0] = matrix[:, 2]
        new_matrix[:, 1] = -matrix[:, 1]
        new_matrix[:, 2] = matrix[:, 0]

        ## Filter the grasp pose
        if  sum(new_matrix[:, 2] * np.array([0,0,-1,0])) < 0.75:  # less means more vertical 
            print('give up {0} grasp'.format(id_num))
            continue

        if abs(sum(new_matrix[:, 1] * np.array([0,0,-1,0]))) > 0.25:   # less means not xie
            print('give up {0} grasp'.format(id_num))
            continue

        q = quaternion_from_matrix(new_matrix)
        qs.append(q)

        offset_pre = -0.1
        t_pre = t + new_matrix[0:3, 2] * offset_pre
        ts_pre.append(t_pre)

        offset_grasp = -0.05
        t_grasp = t + new_matrix[0:3, 2] * offset_grasp
        ts_grasp.append(t_grasp)

        

    print("Publish the sample point")
    rospy.set_param('good_grasps', len(qs))
    print("Find {0} grasps!".format(len(qs)))
    count_down_clock(3)

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