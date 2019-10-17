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

# Bit operations
BIT_MOVE_16 = 2 ** 16
BIT_MOVE_8 = 2 ** 8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 & 0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


def segmented_led(pointcloud2):
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
            if (math.sqrt(x ** 2 + y ** 2 + z ** 2) < 1 and r / (r + g + b) > 0.9):
                segmented_points.append([x, y, z])
                segmented_colors.append([r, g, b])
                print ("This is it: ", x, y, z, r, g, b)

        Segmented_Scene_Cloud.points = open3d.utility.Vector3dVector(np.array(segmented_points))
        Segmented_Scene_Cloud.colors = open3d.utility.Vector3dVector(np.array(segmented_colors) / 255.0)
        Whole_Scene_Cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        Whole_Scene_Cloud.colors = open3d.utility.Vector3dVector(np.array(rgb) / 255)

    return Whole_Scene_Cloud, Segmented_Scene_Cloud


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    open3d.visualization.draw_geometries([source_temp, target_temp])


if __name__ == "__main__":
    rospy.init_node('cloud_filtering_and_stitching', anonymous=True)

    # -- Read point cloud from file
    import os

    PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__)) + "/"

    # -- Set publishers
    topic_name2 = "/camera2/depth/color/points"
    # pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1)

    # -- Set subscribers
    global received_ros_cloud2
    received_ros_cloud2 = None
    global have_received_a_cam2_frame
    have_received_a_cam2_frame = 0


    # Callbacks:
    def pointcloud_callback2(ros_cloud):
        global have_received_a_cam2_frame
        if have_received_a_cam2_frame == 0:
            global received_ros_cloud2
            received_ros_cloud2 = ros_cloud
            rospy.loginfo("-- Received ROS PointCloud2 message from camera #2.")
            have_received_a_cam2_frame = 1

    # Assigning the subscribers:
    rospy.Subscriber(topic_name2, PointCloud2, pointcloud_callback2)

    while received_ros_cloud2 is None and not rospy.is_shutdown():
        rospy.loginfo("-- Not receiving ROS PointCloud2 message yet ...")

    time.sleep(4)
    # -- After the reception of a PointCloud2-type point cloud, convert it back to open3d PointCloud() class, and draw  it
    Whole_Scene_Cloud, Segmented_Scene_Cloud = segmented_led(received_ros_cloud2)
    print(Segmented_Scene_Cloud)

    # write to file
    Whole_Scene_Filename = PYTHON_FILE_PATH + "Whole_Scene_Camera2.pcd"
    open3d.io.write_point_cloud(Whole_Scene_Filename, Whole_Scene_Cloud)
    rospy.loginfo("-- Write result point cloud to: " + Whole_Scene_Filename)
    output_filename3 = PYTHON_FILE_PATH + "Segmented_Scene_Camera2.pcd"
    open3d.io.write_point_cloud(output_filename3, Segmented_Scene_Cloud)
    rospy.loginfo("-- Write result point cloud to: " + output_filename3)

    # draw
    open3d.visualization.draw_geometries([Whole_Scene_Cloud])
    open3d.visualization.draw_geometries([Segmented_Scene_Cloud])
    rospy.loginfo("-- Finish display. The program is terminating ...\n")
