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
from utils import count_down_clock

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


def send_request(msg):
    print("Starting request node... ")
    print("Subscribing to /camera/depth_registered/points... ")
    print("Send cloud data to remote")
    count_down_clock(4)
    s = requests.Session()
    payload = pickle.dumps({'data' : msg})
    url = 'http://104.196.255.235:8000'
    headers = {'Content-Type': 'application/octet-stream'}
    response = s.post(url, headers=headers, data=payload)
    response_payload = response.content
    grasps = pickle.loads(response_payload)
    for grasp in grasps:
        print('################################')
        print(grasp)
        print('################################')
    testGrasp_point = transformGrasp(grasps)
    print("Subscribed to /camera/depth_registered/points...")


def transformGrasp(graspConfigs):

    rate = rospy.Rate(10)
    tf_listener = tf.TransformListener(rospy.Duration(1))
    br = tf.TransformBroadcaster()
    tf_base_camera = TransformStamped()     # tf::StampedTransform tf_base_odom;

    qs = []
    ts = []
    t_pres = []
    for i, graspConfig in enumerate(graspConfigs):

        approach_x = np.array([graspConfig.approach.x, graspConfig.approach.y, graspConfig.approach.z, 0])
        binormal_y = np.array([graspConfig.binormal.x, graspConfig.binormal.y, graspConfig.binormal.z, 0])
        axis_z = np.array([graspConfig.axis.x, graspConfig.axis.y, graspConfig.axis.z, 0])
        padding = np.array([0,0,0,1])

        rotation_matrix = np.array([approach_x, binormal_y, axis_z, padding]).T
        q = quaternion_from_matrix(rotation_matrix)
        t = np.array([graspConfig.top.x, graspConfig.top.y , graspConfig.top.z ])
        qs.append(q)
        ts.append(t)

    print("Boardcast raw grasps in camera_rgb_optical_frame...")
    rospy.set_param('broad_grasp_camera', 1)

    while not rospy.is_shutdown():
        if rospy.get_param('get_grasp_camera'):
            count_down_clock(3)
            rospy.set_param('get_grasp_camera', 0)
            break
        for i in range(len(qs)):
            br.sendTransform(ts[i],
                            qs[i],
                            rospy.Time.now(),
                            "grasp_"+str(i),
                            "camera_rgb_optical_frame")
        rate.sleep()
