#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs
from geometry_msgs.msg import *
import numpy as np
from gripper import open_gripper, close_gripper
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION
import tf2_geometry_msgs
from tf.transformations import quaternion_from_matrix, quaternion_matrix


from std_msgs.msg import Header
import tf
import tf2_ros
from tutorial import MoveGroupPythonIntefaceTutorial


""" 
Run 1)  rosrun baxter_tools enable_robot.py -e
Run 2) rosrun baxter_interface joint_trajectory_action_server.py
Run 3) roslaunch baxter_moveit_config baxter_grippers.launch
"""
rospy.init_node('move_group_node',
                anonymous=True)
print("Getting robot state... ")
rs = baxter_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled
left = baxter_interface.Gripper('left', CHECK_VERSION)

moveit_commander.roscpp_initialize(sys.argv)

# spinner = rospy.AsyncSpinner(1)
# spinner.start()


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=10)

group = moveit_commander.MoveGroupCommander("left_arm")
group.set_planning_time(20)
group.allow_replanning(True)

tf_listener = tf.TransformListener(rospy.Duration(1))

id_num = raw_input("Input Grasp ID: ")

tf_listener.waitForTransform('base',  "sample_grasp_" + str(id_num), rospy.Time(0), rospy.Duration(2.0))
t, q = tf_listener.lookupTransform("base", "sample_grasp_" + str(id_num), rospy.Time())

matrix = quaternion_matrix(q)
new_matrix = matrix

offset = -0.3
t_pre = t + new_matrix[0:3, 2] * offset

q = quaternion_from_matrix(new_matrix)


tf_listener.waitForTransform('base',  "sample_grasp_" + str(id_num), rospy.Time(0), rospy.Duration(2.0))
t, q = tf_listener.lookupTransform("base", "sample_grasp_" + str(id_num), rospy.Time())
group.clear_pose_targets()
matrix = quaternion_matrix(q)
new_matrix = np.eye(4)
new_matrix[:, 0] = matrix[:, 2]
new_matrix[:, 1] = -matrix[:, 1]
new_matrix[:, 2] = matrix[:, 0]

offset = -0.3
t_pre = t + new_matrix[0:3, 2] * offset
quaternion_from_matrix
print("try to move to object...........Move it!")

q = quaternion_from_matrix(new_matrix)

move_group = MoveGroupPythonIntefaceTutorial()
while True:
    try:
        # waypoints_picking = []
        # # waypoints_picking.append(group.get_current_pose().pose)
        # wpose = geometry_msgs.msg.Pose()
        # wpose.orientation = Quaternion(q[0],q[1], q[2], q[3])
        # wpose.position.x = t_pre[0]
        # wpose.position.y = t_pre[1]
        # wpose.position.z = t_pre[2]
        # waypoints_picking.append(wpose)

        # wpose1 = geometry_msgs.msg.Pose()
        # wpose1.orientation = Quaternion(q[0],q[1], q[2], q[3])
        # wpose1.position.x = t[0]
        # wpose1.position.y = t[1]
        # wpose1.position.z = t[2]
        # waypoints_picking.append(wpose1)
        move_group.plan_cartesian_path( q, t_pre, t)

        # while(1):

        #     (plan, fraction) = group.compute_cartesian_path(waypoints_picking, 0.001, 0)
        #     print(plan)
        #     rospy.sleep(5)

        # print "============ Visualizing plan1"

        # display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        # display_trajectory.trajectory_start = robot.get_current_state()
        # display_trajectory.trajectory.append(plan)
        # display_trajectory_publisher.publish(display_trajectory)
        # print "============ Waiting while plan1 is visualized (again)..."
        # rospy.sleep(5)

        # print(plan)



        # raw_input("Press <Enter> to PRE grasp pose: ")
        print("PRE grasp pose")

    except Exception as e:
        print(e)
    # else:
    #     break


# print("Planning in Rviz")
# for i in range(15):
#     print(15-i)   
#     rospy.sleep(1)

# ###########
# # Gripper #
# ###########
open_gripper(left)
close_gripper(left)

for i in range(3):
    print(3-i)   
    rospy.sleep(1)


while True:
    try:
        waypoints_lifting = []
        waypoints_lifting.append(wpose1)

        wpose2 = geometry_msgs.msg.Pose()
        wpose2.orientation = Quaternion(q[0],q[1], q[2], q[3])
        wpose2.position.x = t_pre[0]
        wpose2.position.y = t_pre[1]
        wpose2.position.z = 0.2
        waypoints_lifting.append(wpose2)

        # wpose3 = geometry_msgs.msg.Pose()
        # wpose3.orientation = Quaternion(q[0],q[1], q[2], q[3])
        # wpose3.position.x = 0.408
        # wpose3.position.y = 0.791
        # wpose3.position.z = 0.5
        # waypoints_lifting.append(wpose3)

        # [0.551, 0.690, -0.049]
        wpose4 = geometry_msgs.msg.Pose()
        wpose4.orientation = Quaternion(q[0],q[1], q[2], q[3])
        wpose4.position.x = 0.551
        wpose4.position.y = 0.690
        wpose4.position.z = -0.049
        waypoints_lifting.append(wpose4)

        (plan, fraction) = group.compute_cartesian_path(waypoints_lifting, 0.01, 0)
        print "============ Visualizing plan1"
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        display_trajectory_publisher.publish(display_trajectory)
        print "============ Waiting while plan1 is visualized (again)..."
        rospy.sleep(5)
        # print("Planning in Rviz")
        print(plan)
        rospy.sleep(6)

        raw_input("Press <Enter> to grasp pose: ")
        print("PRE grasp pose")

    except Exception as e:
        print(e)
    else:
        break

###########
# Gripper #
###########
open_gripper(left)

for i in range(3):
    print(3-i)   
    rospy.sleep(1)
print('Success!!')






# for i in range(25):
#     print(25-i)   
#     rospy.sleep(1)











