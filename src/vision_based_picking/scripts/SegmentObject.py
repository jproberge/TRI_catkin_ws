#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author: Jean-Philippe ROberge
Date: October 10th 2019
This script contains 2 functions for converting cloud format between Open3D and ROS:   
* convertCloudFromOpen3dToRos  
* convertCloudFromRosToOpen3d
where the ROS format refers to "sensor_msgs/PointCloud2.msg" type.

This script also contains a test case, which does such a thing:  
(1) Acquires frames from two cameras
(2)

'''

from __future__ import division
import open3d
import numpy as np
import copy
from ctypes import *  # convert float to uint32

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from vision_based_picking.srv import Acquire, AcquireResponse
import sensor_msgs.point_cloud2 as pc2
import math
import time


# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
                [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]
CLIPPING_DISTANCE = 1 # for calibration only, in meter
RED_RATIO_THRESHOLD = 0.8 # Experimentally, this value seems to be the best trade-off

# Bit operations
BIT_MOVE_16 = 2 ** 16
BIT_MOVE_8 = 2 ** 8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 & 0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


def segmented_object(pointcloud2):
    # Get cloud data from ros_cloud
    field_names = [field.name for field in pointcloud2.fields]
    cloud_data = list(pc2.read_points(pointcloud2, skip_nans=True, field_names=field_names))

    Segmented_Scene_Cloud = open3d.geometry.PointCloud()
    Whole_Scene_Cloud = open3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

        # Get xyz
        xyz = [(x, y, z) for x, y, z, rgb in cloud_data]

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD]) == float:  # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data]
        # combine

        segmented_points = []
        segmented_colors = []
        PointsXYZRGB = np.concatenate((np.array(xyz), np.array(rgb)), axis=1)
        for x, y, z, r, g, b in PointsXYZRGB:
            if math.sqrt(x ** 2 + y ** 2 + z ** 2) < CLIPPING_DISTANCE and r / (r + g + b) > RED_RATIO_THRESHOLD:
                segmented_points.append([x, y, z])
                segmented_colors.append([r, g, b])
                print ("This is it: ", x, y, z, r, g, b)

        if not len(segmented_points) == 0:
            Segmented_Scene_Cloud.points = open3d.utility.Vector3dVector(np.array(segmented_points))
            Segmented_Scene_Cloud.colors = open3d.utility.Vector3dVector(np.array(segmented_colors) / 255.0)

        Whole_Scene_Cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        Whole_Scene_Cloud.colors = open3d.utility.Vector3dVector(np.array(rgb) / 255)

    return Whole_Scene_Cloud, Segmented_Scene_Cloud


def handle_acquire(req):
    global CurrentRequest
    global have_received_a_cam_frame
    if req.the_request == "Start":
        CurrentRequest = "Start"
        print "We have receivend the request=", req.the_request, " with cam_index=", req.cam_index
        return AcquireResponse(0)
    elif req.the_request == "Shutdown":
        CurrentRequest = "Shutdown"
        print "We have receivend the request=", req.the_request, " with cam_index=", req.cam_index
        return AcquireResponse(0)
    else:
        CurrentRequest = "Stop"
        have_received_a_cam_frame = 0
        print "We have receivend the request=", req.the_request, " with cam_index=", req.cam_index
        return AcquireResponse(0)


if __name__ == "__main__":
    rospy.init_node('acquire_scene', anonymous=True)
    s = rospy.Service('acquire', Acquire, handle_acquire)
    import os
    PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__)) + "/"



    # -- Set some global variables
    global received_ros_cloud2_c1
    received_ros_cloud2_c1 = None
    global received_ros_cloud2_c2
    received_ros_cloud2_c2 = None
    global have_received_a_cam_frame
    have_received_a_cam_frame = 0
    global CurrentRequest
    CurrentRequest = "Start"

    # Callbacks:
    def pointcloud_callback_c1(ros_cloud1):
        print(ros_cloud1)

    def pointcloud_callback_c2(ros_cloud2):
        print(ros_cloud2)


    while not rospy.is_shutdown():
        if CurrentRequest == "Start":

            # -- Set subscribers
            topic_name1 = "/camera1/depth/color/points"
            topic_name2 = "/camera2/depth/color/points"

            camera1 = rospy.Subscriber(topic_name1, PointCloud2, pointcloud_callback_c1)
            camera2 = rospy.Subscriber(topic_name2, PointCloud2, pointcloud_callback_c2)

            while received_ros_cloud2_c1 is None and not rospy.is_shutdown():
                rospy.loginfo("-- Not receiving ROS PointCloud2 from camera 1 yet ...")

            # Whole_Scene_Cloud_c1, Segmented_Scene_Cloud_c1 = segmented_object(received_ros_cloud2_c1)
            # print(Whole_Scene_Cloud_c1)
            time.sleep(1)

            while received_ros_cloud2_c2 is None and not rospy.is_shutdown():
                rospy.loginfo("-- Not receiving ROS PointCloud2 from camera 2 yet ...")

            time.sleep(1)

            print "here"
            Whole_Scene_Cloud_c2, Segmented_Scene_Cloud_c2 = segmented_object(received_ros_cloud2_c2)
            print(Whole_Scene_Cloud_c2)

            # write to file
            Whole_Scene_Filename_c1 = PYTHON_FILE_PATH + "Whole_Scene_Camera1.pcd"
            open3d.io.write_point_cloud(Whole_Scene_Filename_c1, Whole_Scene_Cloud_c1)
            rospy.loginfo("-- Write result point cloud to: " + Whole_Scene_Filename_c1)
            if not Segmented_Scene_Cloud_c1 == []:
                Segmented_Scene_Filename_c1 = PYTHON_FILE_PATH + "Segmented_Scene_Camera1.pcd"
                open3d.io.write_point_cloud(Segmented_Scene_Filename_c1, Segmented_Scene_Cloud_c1)
                rospy.loginfo("-- Write result point cloud to: " + Segmented_Scene_Filename_c1)

            # CurrentRequest = "Stop"
            have_received_a_cam_frame = 0
        elif CurrentRequest == "Shutdown":
            time.sleep(1)
            exit(0)