#!/usr/bin/env python
import numpy as np
import urllib2
import rospy
import requests
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import pickle
from gpd.msg import GraspConfig
from geometry_msgs.msg import *

# For transform grasp
from std_msgs.msg import Header
import tf
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_matrix, quaternion_matrix

# For movement planning
import sys
import copy
import moveit_commander
import moveit_msgs.msg


def transformGrasp(graspConfigs):
    # GraspPose grasp_pose;
    #   tf::StampedTransform tf_base_odom;
    #   // TODO: try transformPose : http://docs.ros.org/indigo/api/tf/html/c++/classtf_1_1Transformer.html#a0a7b72eb4cc62194164d030afbefda9a
    #   tf::Quaternion q_grasp_base;
    #   tf::Matrix3x3 rot_matrix_grasp_base(-grasp_msg.axis.x, grasp_msg.binormal.x, grasp_msg.approach.x,
    #                                       -grasp_msg.axis.y, grasp_msg.binormal.y, grasp_msg.approach.y,
    #                                       -grasp_msg.axis.z, grasp_msg.binormal.z, grasp_msg.approach.z);

    #   tf::Vector3 tr_grasp_base(grasp_msg.bottom.x, grasp_msg.bottom.y, grasp_msg.bottom.z);
    #   tf::Transform tf_grasp_base(rot_matrix_grasp_base, tr_grasp_base);

    tf_listener = tf.TransformListener(rospy.Duration(1))
    tf_base_camera = TransformStamped()     # tf::StampedTransform tf_base_odom;


    id_list = ['_1', '_2', '_3', '_4', '_5', '_6', '_7', '_8', '_9', '_10', '_11', '_12', '_13', '_14', '_15', '_16', '_17', '_18', '_19', '_20']
    qs = []
    ts = []
    t_pres = []
    for i, graspConfig in enumerate(graspConfigs):
        # transform = tf_listener.lookupTransform("camera_link", "base", rospy.Time())
        # header = Header(frame_id='camera_link', stamp=rospy.Time.now())
        # bottom_point_msg = PointStamped(Header(frame_id='camera_link', stamp=rospy.Time.now()), graspConfig.bottom)
        # transformed_top = PointStamped(Header(frame_id='camera_link', stamp=rospy.Time.now()), graspConfig.top)
        # transformed_surface = PointStamped(Header(frame_id='camera_link', stamp=rospy.Time.now()), graspConfig.surface)
        
        # transformed_approach = Vector3Stamped(Header(frame_id='camera_link', stamp=rospy.Time.now()), graspConfig.approach)
        # transformed_binormal = Vector3Stamped(Header(frame_id='camera_link', stamp=rospy.Time.now()), graspConfig.binormal)
        # transformed_axis = Vector3Stamped(Header(frame_id='camera_link', stamp=rospy.Time.now()), graspConfig.axis)

        # transformed_sample = PointStamped(Header(frame_id='camera_link', stamp=rospy.Time.now()), graspConfig.sample)



        approach_x = np.array([graspConfig.approach.x, graspConfig.approach.y, graspConfig.approach.z, 0])
        binormal_y = np.array([graspConfig.binormal.x, graspConfig.binormal.y, graspConfig.binormal.z, 0])
        axis_z = np.array([graspConfig.axis.x, graspConfig.axis.y, graspConfig.axis.z, 0])
        padding = np.array([0,0,0,1])

        rotation_matrix = np.array([approach_x, binormal_y, axis_z, padding]).T
        q = quaternion_from_matrix(rotation_matrix)

        # grasp_offset_ = 1

        # graspConfig.top.x = graspConfig.top.x + grasp_offset_ * graspConfig.approach.x
        # graspConfig.top.y = graspConfig.top.y + grasp_offset_ * graspConfig.approach.y
        # graspConfig.top.z = graspConfig.top.z + grasp_offset_ * graspConfig.approach.z

        # pre_grasp_offset_ = -0.07
        # pre_x = graspConfig.top.x + pre_grasp_offset_ * graspConfig.approach.x
        # pre_y = graspConfig.top.y + pre_grasp_offset_ * graspConfig.approach.y
        # pre_z = graspConfig.top.z + pre_grasp_offset_ * graspConfig.approach.z

        grasp_offset_ = 0   
        x = graspConfig.top.x + grasp_offset_ * graspConfig.approach.x
        y = graspConfig.top.y + grasp_offset_ * graspConfig.approach.y
        z = graspConfig.top.z + grasp_offset_ * graspConfig.approach.z


        # t = np.array([graspConfig.sample.x, graspConfig.sample.y, graspConfig.sample.z]).T # graspConfig.sample.z
        # t_pre = np.array([pre_x, pre_y, pre_z]).T 
        t = np.array([x, y, z])
        qs.append(q)
        # t_pres.append(t_pre)
        ts.append(t)

        # try:
        #     tf_listener.waitForTransform('base', 'camera_link', rospy.Time(0), rospy.Duration(3.0))
        #     (trans, quat) = tf_listener.lookupTransform("/base", "/camera_link", rospy.Time())
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
        #     print "Fail", 
        
        # print("##################################")
        # print(trans)
        # print(type(trans))
        # print(quat)


        # homo = quaternion_matrix(quat)
        # grasp = quaternion_matrix(q)
        # grasp[0:3, 3] = t


        # print(homo)
        # homo[0:3, 3] = trans
        # grasp_transed = np.dot(homo, grasp)
        # grasp_transed_rot = np.array(grasp_transed)
        # grasp_transed_rot[0:3, 3] = np.array([0,0,0]) 

        # q_transformed = quaternion_from_matrix(grasp_transed)
        # t_transformed = grasp_transed[0:3,3]


        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10)
        
    raw_input("Press enter to publish the sample point")
    while not rospy.is_shutdown():
        for i in range(len(qs)):
            # br.sendTransform(t_pres[i],
            #                 qs[i],
            #                 rospy.Time.now(),
            #                 "sample_pregrasp"+id_list[i],
            #                 "camera_rgb_optical_frame")

            br.sendTransform(ts[i],
                            qs[i],
                            rospy.Time.now(),
                            "grasp"+id_list[i],
                            "camera_rgb_optical_frame")
        rate.sleep()




    # Simply return the bottom pose of the robot hand for now, for testing
    return transformed_bottom_msg.point
    


def cloud_callback(msg):
    raw_input("Press enter to send cloud data to remote")
    s = requests.Session()
    payload = pickle.dumps({'data' : msg})
    url = 'http://104.196.255.235:8000'
    headers = {'Content-Type': 'application/octet-stream'}
    # req = urllib2.Request('http://104.196.255.235:8000', times=None)
    # req.add_header('Content-Type', 'application/json')
    # req.add_header('Content-Type', 'application/octet-stream')
    # response = urllib2.urlopen(req, payload)
    response = s.post(url, headers=headers, data=payload)
    response_payload = response.content
    grasps = pickle.loads(response_payload)
    for grasp in grasps:
        print('################################')
        print(grasp)
        print('################################')



    testGrasp_point = transformGrasp(grasps) 
    # print(testGrasp_point)
    poseTarget = Pose()
    poseTarget.position = testGrasp_point
    # poseTarget.orientation.w = 1.0
    group.set_pose_target(poseTarget)


    plan1 = group.plan()
    group.go(wait=True)

    # print(response_payload)
    # print(response)

def request_grasp():
    print("Starting request node... ")
    rospy.init_node('request_grasp')
    print("Subscribing to /camera/depth_registered/points... ")
    cloud_subscribe = rospy.Subscriber('/processed_points', PointCloud2, cloud_callback)
    print("Subscribed to /camera/depth_registered/points...")
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('request_grasp')
    
    # # Set up moveIt
    # moveit_commander.roscpp_initialize(sys.argv)

    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # display_trajectory_publisher = rospy.Publisher(
    #                                     '/move_group/display_planned_path',
    #                                     moveit_msgs.msg.DisplayTrajectory, queue_size=0)

    # group = moveit_commander.MoveGroupCommander("left_arm")

    # display_trajectory_publisher = rospy.Publisher(
    #                                     '/move_group/display_planned_path',
    #                                     moveit_msgs.msg.DisplayTrajectory)

    request_grasp()


# import requests
# post_data = {"amount":10000, "service":"writing blog posts"}

# r = requests.post('http://example.com/api', post_data, auth=('user', 'pass'))

# print r.status_code
# print r.headers['content-type']
