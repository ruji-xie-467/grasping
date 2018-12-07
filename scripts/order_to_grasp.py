#!/usr/bin/env python

import sys
import rospy
from PrepareGrasp.srv import *
items = ['apples', 'bananas', 'oranges']

def prepare_grasp_client(input_list):
    rospy.wait_for_service('prepare_grasp_service')
    try:
        prepare_grasp_handle = rospy.ServiceProxy('prepare_grasp_service', PrepareGrasp)
        response = prepare_grasp_handle(input_list) #PointCloud2[] clusters
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


# Returns true on success, false on failure
# Takes order from user and moves the order to grasp
def order_to_grasps():
    # TODO: Prompt user for their order. i.e. # bananas, #apples, # oranges
    order_list = []
    for item in items:
        num_fruit = int(raw_input("How many {fruit} would you like to order?: ".format(fruit=item)))
        order_list += [item] * num_fruit
    
    cluster_list = prepare_grasp_client(input_list)

    for cluster in cluster_list:

