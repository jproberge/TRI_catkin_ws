#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Author: Michael Lin
Date: November 1st 2019
This is a simple ros service script to get a frame of points clouds from intel realsense.
Inspired from SegmentObject.py by JP Roberge.
"""

from __future__ import division
import open3d
import numpy as np
from std_msgs.msg import Header
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from ctypes import *
from vision_based_picking.srv import Acquire, AcquireResponse
import os

def convert_rospc_to_o3dpc(ros_pc):
    """ Function taken and modified from felixchenfy
        https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/blob/master/lib_cloud_conversion_between_Open3D_and_ROS.py
    """
    
    # Get cloud data from ros_pc
    field_names = [field.name for field in ros_pc.fields]
    pc_data = list(pc2.read_points(ros_pc, skip_nans=True, field_names = field_names))

    open3d_cloud = open3d.PointCloud()

    # Check empty
    if len(pc_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in pc_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(pc_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in pc_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in pc_data ]

        # combine
        open3d_cloud.points = open3d.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in pc_data ] # get xyz
        open3d_cloud.points = open3d.Vector3dVector(np.array(xyz))
    return open3d_cloud

def service_request(req_arg):
    global serviceFinished

    # Acquire the scene from both cameras:
    while received_ros_cloud2_c1 is None:
        rospy.loginfo("-- Not receiving ROS PointCloud2 from camera 1 yet ...")
    print "we have received a frame from cam 1"
    o3dpc_fn = DATA_DIR_PATH + req_arg.filename
    open3d.io.write_point_cloud(o3d_fn, received_ros_cloud2_c1)

    serviceFinished = True
    # once we are done return the file path
    return AcquireResponse(o3dpc_fn)

def acquire_bowl_pc():
    rospy.init_node('acquire_bowl', anonymous=True)
    s = rospy.Service('acquire', Acquire, service_request)

    # Set subscribers for camera frames
    topic_name1 = "/camera1/depth/color/points"
    camera1 = rospy.Subscriber(topic_name1, PointCloud2, acquired_pc_c1_callback)

    DATA_DIR_PATH = os.path.join(os.path.dirname(__file__)) + "/data/"

    # -- Set some global variables
    global received_ros_cloud2_c1
    received_ros_cloud2_c1 = None
    global have_received_a_cam_frame_c1   # flag to indicate we got one frame
    have_received_a_cam_frame_c1 = 0
    global serviceFinished
    serviceFinished = False

    while not rospy.is_shutdown():
        if serviceFinished:
            sleep(1)
            exit(0)


# Callbacks:
def acquired_pc_c1_callback(ros_pc):
    global have_received_a_cam_frame_c1
    if have_received_a_cam_frame_c1 == 0:
        global received_ros_cloud2_c1
        received_ros_cloud2_c1=convert_rospc_to_o3dpc(ros_cloud1)
        have_received_a_cam_frame_c1=1

if __name__ == "__main__":
  acquire_bowl_pc()