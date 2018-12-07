#!/usr/bin/env python
"""
Path Planning Script for Lab 8
Author: Valmik Prabhu
"""

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
# from intera_interface import Limb

def main():
    """
    Main Script
    """
    #############
    offset_p = -0.07
    offset_g = 0.01
    #############


    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("left_arm")
    planner.set_goal_tolerance(0.01)  #(0.0001, 0.0001, 0.001)

    ##
    ## Init Gripper
    ##
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left', CHECK_VERSION)


    tf_listener = tf.TransformListener(rospy.Duration(1))

    raw_input("Press <Enter> to start!")
    try_another_grasp = 0
    good_grasps = rospy.get_param('good_grasps')
    for g in range(good_grasps):
        i_pregrasp = 0
        ## get grasp pose
        tf_listener.waitForTransform('base',  "sample_grasp_" + str(g), rospy.Time(0), rospy.Duration(2.0))
        t, q = tf_listener.lookupTransform("base", "sample_grasp_" + str(g), rospy.Time())
        matrix = quaternion_matrix(q)
        new_matrix = np.eye(4)
        new_matrix[:, 0] = matrix[:, 2]
        new_matrix[:, 1] = -matrix[:, 1]
        new_matrix[:, 2] = matrix[:, 0]

        t_pre = t + new_matrix[0:3, 2] * offset_p
        t_g = t + new_matrix[0:3, 2] *offset_g

        quaternion_from_matrix
        q = quaternion_from_matrix(new_matrix)

        print("try grasp {0} to move to object...........Move it!".format(g))


        while not rospy.is_shutdown():
            
            i_pregrasp += 1
            try:
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = t_pre[0]
                goal_1.pose.position.y = t_pre[1]
                goal_1.pose.position.z = t_pre[2]

                #Orientation as a quaternion
                goal_1.pose.orientation = Quaternion(q[0],q[1], q[2], q[3])

                plan = planner.plan_to_pose(goal_1, list())

                ## In Rviz
                # rospy.sleep(6)

                # raw_input("Press <Enter> to PRE grasp pose: ")
                print("PRE grasp pose {0}, try {1} times".format(g, i_pregrasp))
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                ## jump to next grasp pose (1)
                if i_pregrasp > 3:
                    try_another_grasp = 1
                    break
            else:
                break

        ## jump to next grasp pose (2)
        if try_another_grasp == 1:
            print('Try another !')
            try_another_grasp = 0
            planner.clear_goal()
            continue 

        i_grasp = 0 
        while not rospy.is_shutdown():
            i_grasp += 1
            try:
                goal_2 = PoseStamped()
                goal_2.header.frame_id = "base"

                #x, y, and z position
                goal_2.pose.position.x = t_g[0]
                goal_2.pose.position.y = t_g[1]
                goal_2.pose.position.z = t_g[2]

                #Orientation as a quaternion
                goal_2.pose.orientation = Quaternion(q[0],q[1], q[2], q[3])

                plan = planner.plan_to_pose(goal_2, list())

                ## in Rviz
                # rospy.sleep(6)                

                # raw_input("Press <Enter> to GRASP pose : ")
                print("GRASP pose %d" % i_grasp)

                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                ## jump to next grasp pose (1)
                if i_grasp > 3:
                    try_another_grasp = 1
                    break
            else:
                break

        ## jump to next grasp pose (2)
        if try_another_grasp == 1:
            print('Try another !')
            try_another_grasp = 0
            planner.clear_goal()
            continue 

        ##
        ## grasp
        ##
        open_gripper(left)
        close_gripper(left)


        i_leave = 1
        while not rospy.is_shutdown():
            i_leave += 1
            try:
                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"

                #x, y, and z position
                # [0.551, 0.690, -0.049]
                goal_3.pose.position.x = 0.551
                goal_3.pose.position.y = 0.690
                goal_3.pose.position.z = -0.049

                #Orientation as a quaternion
                goal_3.pose.orientation = Quaternion(q[0],q[1], q[2], q[3])


                plan = planner.plan_to_pose(goal_3, list())

                # raw_input("Press <Enter> to LEAVE pose: ")
                print("LEAVE pose %d" % i_leave)

                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

        ##
        ## grasp
        ##
        open_gripper(left)


        raw_input("Press <Enter> to continue")
            

# ###### Lift High ######


#         i_lift = 1
#         while not rospy.is_shutdown():
#             i_lift += 1
#             try:
#                 goal_4 = PoseStamped()
#                 goal_4.header.frame_id = "base"

#                 #x, y, and z position
#                 # [0.551, 0.690, -0.049]
#                 goal_4.pose.position.x = t_pre[0]
#                 goal_4.pose.position.y = t_pre[1]
#                 goal_4.pose.position.z = t_pre[2]

#                 #Orientation as a quaternion
#                 goal_4.pose.orientation = Quaternion(q[0],q[1], q[2], q[3])


#                 plan = planner.plan_to_pose(goal_4, list())

#                 raw_input("Press <Enter> to LEAVE pose: ")
#                 print("LEAVE pose %d" % i_leave)

#                 if not planner.execute_plan(plan):
#                     raise Exception("Execution failed")
#             except Exception as e:
#                 print e
#             else:
#                 break


#         i_leave = 1
#         while not rospy.is_shutdown():
#             i_leave += 0
#             try:
#                 goal_5 = PoseStamped()
#                 goal_5.header.frame_id = "base"

#                 #x, y, and z position
#                 # [0.551, 0.690, -0.049]
#                 goal_5.pose.position.x = 0.551
#                 goal_5.pose.position.y = 0.690
#                 goal_5.pose.position.z = -0.049

#                 #Orientation as a quaternion
#                 goal_5.pose.orientation = Quaternion(q[0],q[1], q[2], q[3])


#                 plan = planner.plan_to_pose(goal_5, list())

#                 raw_input("Press <Enter> to LEAVE pose: ")
#                 print("LEAVE pose %d" % i_leave)

#                 if not planner.execute_plan(plan):
#                     raise Exception("Execution failed")
#             except Exception as e:
#                 print e
#             else:
#                 break



if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
