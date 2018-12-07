#!/usr/bin/env python
import rospy
from threading import Lock, Thread
from clp.msg import *
from detection.msg import *
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point

lock = Lock()
obj_to_point_map = dict()
point_to_cloud_map = dict()

## Given an item finds the closest 
def match_point_to_nearest_cloud(item_point, point_to_cloud_map):
    return point_to_cloud_map[min(point_to_cloud_map.keys(), key=lambda p1: abs(p1-item_point))]

## Prepare grasp service call handler
def handle_prepare_grasp(item_list_msg):
    global obj_to_point_map
    global point_to_cloud_map

    lock.acquire()
    response = PrepareGraspResponse()
    item_list = item_list_msg.items
    for item in item_list:
        if item not in obj_to_point_map.keys():
            return response
        item_point = obj_to_point_map[item].pop(0)
        response.grasp_clouds.append(match_point_to_nearest_cloud(item_point, point_to_cloud_map))
    lock.release()

    return response
    
## Update the object to point position mapping
def obj_to_point_callback(itemListMsg):
    global obj_to_point_map

    lock.acquire()
    table_item_list = itemListMsg.item_list
    ## Get the center of the bounding box? Or is it already stored in the message?
    for item in table_item_list:
        obj_to_point[item.type].append(item.center)
    lock.release()

## Update the point position to point cloud cluster mapping
def point_to_cloud_callback(clusterListMsg):
    global point_to_cloud_map

    lock.acquire()
    cluster_list = clusterListMsg.cluster_list
    point_to_cloud_map = {cluster.cloud_center : cluster.cloud for cluster in cluster_list}
    lock.release()

## The grasp server listens to both the sensing modules
## and keeps updated mapping of the objects on the scene
def prepare_grasp_server() :
    rospy.init_node('prepare_grasp_server')
    s = rospy.service('prepare_grasp_service', PrepareGrasp, handle_prepare_grasp)
    # Keep the queue size as 1 for both topics
    obj_to_point_sub = rospy.Subscriber('obj_to_point', ItemList, obj_to_point_callback)
    point_to_cloud_sub = rospy.Subscriber('point_to_cloud', ClusterList, point_to_cloud_callback)
    rospy.spin()


if __name__ == "__main__":
    prepare_grasp_server()