#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import *
import numpy as np

from baxter_interface import Gripper


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
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=0)

group = moveit_commander.MoveGroupCommander("left_arm")
group.set_planning_time(20)

display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

# x: 0.42236671404599313
# y: -0.19620172593347457
# z: -0.0023399755500243535



tf_listener = tf.TransformListener(rospy.Duration(1))
id_num = raw_input("Grasp number: ")

t, q = tf_listener.lookupTransform("base", "sample_grasp_" + id_num, rospy.Time())


 

matrix = quaternion_matrix(q)
new_matrix = np.eye(4)

# new_matrix[0:2, 0] = matrix[0:2, 1]
# new_matrix[0:2, 1] = matrix[0:2, 2]

new_matrix[:, 0] = matrix[:, 1]
new_matrix[:, 1] = matrix[:, 2]
new_matrix[:, 2] = matrix[:, 0]
# new_matrix[:, 2] = np.array([0,0,-1,0])
raw_input("New_matrix")
print(new_matrix)
q = quaternion_from_matrix(new_matrix)


br = tf.TransformBroadcaster()
rate = rospy.Rate(10)
    
# raw_input("Press enter to publish the sample point")
# while not rospy.is_shutdown():
#     # br.sendTransform(ts[i],
#     #                 qs[i],
#     #                 rospy.Time.now(),
#     #                 "sample_grasp_base"+id_list[i],
#     #                 "base")

#     br.sendTransform(t,
#                     q,
#                     rospy.Time.now(),
#                     "a_try_grasp",
#                     "base")
#     rate.sleep()

pose_target = geometry_msgs.msg.Pose()
print(t)
# print(type(t))
pose_target.position = Point(t[0], t[1], t[2])
pose_target.orientation = Quaternion(q[0], q[1], q[2], q[3])

# pose_target.orientation = Quaternion(0, 0, 0, 1)
group.set_pose_target(pose_target)

plan1 = group.plan()
group.go(wait=True)