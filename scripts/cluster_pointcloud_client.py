#!/usr/bin/env python3

import sys
import rospy
from std_srvs.srv import Trigger

def call_pointcloud_cluster_service():
    print ("Requesting point cloud filtering")
    rospy.wait_for_service('/cluster_pointcloud')
    try:
        service_proxy = rospy.ServiceProxy('/cluster_pointcloud', Trigger)
        resp1 = service_proxy()
        print ("Service invoked")
        return resp1
    except rospy.ServiceException as e:
        print ("Service call failed: %s" %e)

if __name__ == "__main__":
    call_pointcloud_cluster_service()
