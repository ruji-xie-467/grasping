#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from baxter_interface import Gripper

""" 
Run 1)  rosrun baxter_tools enable_robot.py -e
Run 2) rosrun baxter_interface joint_trajectory_action_server.py
Run 3) roslaunch baxter_moveit_config baxter_grippers.launch
"""

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=0)

group = moveit_commander.MoveGroupCommander("left_arm")

# display_trajectory_publisher = rospy.Publisher(
#                                     '/move_group/display_planned_path',
#                                     moveit_msgs.msg.DisplayTrajectory)

# x: 0.42236671404599313
# y: -0.19620172593347457
# z: -0.0023399755500243535

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 1
pose_target.position.x = .5
pose_target.position.y = .5
pose_target.position.z = 0
group.set_pose_target(pose_target)

plan1 = group.plan()
group.go(wait=True)