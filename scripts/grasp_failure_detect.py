#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import *
import numpy as np
from gripper import open_gripper, close_gripper
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION

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
# group.allow_replanning(True)

tf_listener = tf.TransformListener(rospy.Duration(1))

## pose_goal = geometry_msgs.msg.Pose()
## pose_goal.orientation = Quaternion(q[0],q[1], q[2], q[3])
## pose_goal.position = Point(t_pre[0], t_pre[1], t_pre[2])
## group.set_pose_target(pose_goal)

## plan = group.plan()
## print(plan)
## while not rospy.is_shutdown():
##     group.plan()
##     rospy.sleep(6)

## pose_goal = geometry_msgs.msg.Pose()
## pose_goal.orientation = Quaternion(q[0],q[1], q[2], q[3])
## pose_goal.position = Point(t[0], t[1], t[2])
## group.set_pose_target(pose_goal)

table_x_width = 1
table_y_length = 1.2
table_z_thick = 0.06    
table_position_x = 0.8
table_position_y = 0
table_position_z = -0.3

scene.remove_world_object('box')
table_pose = geometry_msgs.msg.PoseStamped()
table_pose.header.frame_id = "base"
table_pose.pose.orientation.w = 1.0
table_pose.pose.position.x = table_position_x
table_pose.pose.position.z = table_position_z
box_name = "table"
scene.add_box(box_name, table_pose, size=(table_x_width, table_y_length, table_z_thick))

center_of_table_plane = [table_position_x+table_x_width/2,  \
                         table_position_y+table_y_length/2, \
                         table_position_z+table_z_thick/2]

kinect_pose = geometry_msgs.msg.PoseStamped()
kinect_pose.header.frame_id = "base"
kinect_pose.pose.orientation.w = 2.0
kinect_pose.pose.position.x = table_position_x + 0.3
kinect_pose.pose.position.y = table_position_y + table_y_length/2-0.2/2
kinect_pose.pose.position.z = 0.4
box_name = "kinect"
scene.add_box(box_name, kinect_pose, size=(0.2,0.2,1))

raw_input("Press [Enter] to start!")
# while(1):
#     pass

good_grasps = rospy.get_param('good_grasps')
if good_grasps == 0:
    print('#### No grasp find! ####')
for ii in range(good_grasps):
    tf_listener.waitForTransform('base',  "sample_grasp_" + str(ii), rospy.Time(0), rospy.Duration(2.0))
    t, q = tf_listener.lookupTransform("base", "sample_grasp_" + str(ii), rospy.Time())
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


    waypoints_picking = []
    # waypoints_picking.append(group.get_current_pose().pose)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation = Quaternion(q[0],q[1], q[2], q[3])
    wpose.position.x = t_pre[0]
    wpose.position.y = t_pre[1]
    wpose.position.z = t_pre[2]
    waypoints_picking.append(wpose)

    wpose1 = geometry_msgs.msg.Pose()
    wpose1.orientation = Quaternion(q[0],q[1], q[2], q[3])
    wpose1.position.x = t[0]
    wpose1.position.y = t[1]
    wpose1.position.z = t[2]
    waypoints_picking.append(wpose1)

    (plan, fraction) = group.compute_cartesian_path(waypoints_picking, 0.002, 0.05)
    print(plan)

    if not plan.joint_trajectory.points[0].velocities: 
        print('plan {} fail in picking!'.format(ii))
        continue
    
    group.execute(plan, wait =True)

    # print("Planning in Rviz")
    # for i in range(15):
    #     print(15-i)   
    #     rospy.sleep(1)

    ###########
    # Gripper #
    ###########
    open_gripper(left)
    close_gripper(left)

    for i in range(3):
        print(3-i)   
        rospy.sleep(1)


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

    # [0.408, 0.791, -0.188]
    wpose4 = geometry_msgs.msg.Pose()
    wpose4.orientation = Quaternion(q[0],q[1], q[2], q[3])
    wpose4.position.x = 0.408
    wpose4.position.y = 0.791
    wpose4.position.z = -0.188
    waypoints_lifting.append(wpose4)

    (plan, fraction) = group.compute_cartesian_path(waypoints_lifting, 0.002, 0.05)
    print(plan)
    if not plan.joint_trajectory.points[0].velocities: 
        print('plan {} fail in placing!'.format(ii))
        waypoints_fail = []
        wpose4 = geometry_msgs.msg.Pose()
        wpose4.orientation = Quaternion(q[0],q[1], q[2], q[3])
        wpose4.position.x = 0.408
        wpose4.position.y = 0.791
        wpose4.position.z = -0.188
        waypoints_fail.append(wpose4)
        (plan, fraction) = group.compute_cartesian_path(waypoints_lifting, 0.002, 0)
        if not plan.joint_trajectory.points[0].velocities:
            print('Cannot work correctly.')
            continue
        group.execute(plan, wait =True)
        open_gripper(left)
        continue

    group.execute(plan, wait =True)



    # print("Planning in Rviz")
    # for i in range(25):
    #     print(25-i)   
    #     rospy.sleep(1)

    ###########
    # Gripper #
    ###########
    open_gripper(left)

    for i in range(3):
        print(3-i)   
        rospy.sleep(1)
    print('Success!!')
    break














# (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0)







