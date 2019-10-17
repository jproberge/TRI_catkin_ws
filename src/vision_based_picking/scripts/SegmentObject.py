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
from std_msgs.msg import Header
import rospy
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
CLIPPING_DISTANCE = 0.9 # for calibration only, in meter
COLOR_THRESHOLD = 0.5 # Experimentally, this value seems to be a good trade-off

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
            if math.sqrt(x ** 2 + y ** 2 + z ** 2) < CLIPPING_DISTANCE and b / (r + g + b) > COLOR_THRESHOLD:
                segmented_points.append([x, y, z])
                segmented_colors.append([r, g, b])
                print ("This is it: ", x, y, z, r, g, b)

        if not len(segmented_points) == 0:
            Segmented_Scene_Cloud.points = open3d.utility.Vector3dVector(np.array(segmented_points))
            Segmented_Scene_Cloud.colors = open3d.utility.Vector3dVector(np.array(segmented_colors) / 255.0)

        Whole_Scene_Cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        Whole_Scene_Cloud.colors = open3d.utility.Vector3dVector(np.array(rgb) / 255)

    return Segmented_Scene_Cloud


def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points = np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors:  # XYZ only
        fields = FIELDS_XYZ
        cloud_data = points
    else:  # XYZ + RGB
        fields = FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors) * 255)  # nx3 matrix
        colors = colors[:, 0] * BIT_MOVE_16 + colors[:, 1] * BIT_MOVE_8 + colors[:, 2]
        cloud_data = np.c_[points, colors]

    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)


def convert_ros_to_open3d(pointcloud2):
    # Get cloud data from ros_cloud
    field_names = [field.name for field in pointcloud2.fields]
    cloud_data = list(pc2.read_points(pointcloud2, skip_nans=True, field_names=field_names))

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

        Whole_Scene_Cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        Whole_Scene_Cloud.colors = open3d.utility.Vector3dVector(np.array(rgb) / 255)

    return Whole_Scene_Cloud


def handle_acquire(req):
    global CurrentRequest
    global have_received_a_cam_frame
    if req.the_request == "Start":
        CurrentRequest = "Start"
        print "We have received the request=", req.the_request, " with cam_index=", req.cam_index
        return AcquireResponse(0)
    elif req.the_request == "Shutdown":
        CurrentRequest = "Shutdown"
        print "We have received the request=", req.the_request, " with cam_index=", req.cam_index
        return AcquireResponse(0)
    else:
        CurrentRequest = "Stop"
        have_received_a_cam_frame = 0
        print "We have received the request=", req.the_request, " with cam_index=", req.cam_index
        return AcquireResponse(0)

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    open3d.visualization.draw_geometries([source_temp, target_temp])


def filter_pc(cl, voxel_size, sor_params=None, ror_params=None):
    # sor_params = [nb_neighbors, std_ratio]
    # ror_params = [nb_neighbors, radius]
    cl = cl.voxel_down_sample(voxel_size)
    if sor_params is not None:
        cl, ind = cl.remove_statistical_outlier(sor_params[0], sor_params[1])
    if ror_params is not None:
        cl, ind = cl.remove_radius_outlier(ror_params[0], ror_params[1])

    print("Showing filtered point cloud")
    open3d.visualization.draw_geometries([cl])
    return cl

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
    global have_received_a_cam_frame_c1
    have_received_a_cam_frame_c1 = 0
    global have_received_a_cam_frame_c2
    have_received_a_cam_frame_c2 = 0
    global CurrentRequest
    CurrentRequest = "Start"

    # Callbacks:
    def pointcloud_callback_c1(ros_cloud1):
        global have_received_a_cam_frame_c1
        if have_received_a_cam_frame_c1 == 0:
            global received_ros_cloud2_c1
            received_ros_cloud2_c1=ros_cloud1
            print "callback1"
            have_received_a_cam_frame_c1=1

    def pointcloud_callback_c2(ros_cloud2):
        global have_received_a_cam_frame_c2
        if have_received_a_cam_frame_c2 == 0:
            global received_ros_cloud2_c2
            received_ros_cloud2_c2=ros_cloud2
            print "callback2"
            have_received_a_cam_frame_c2=1


    while not rospy.is_shutdown():
        if CurrentRequest == "Start":

            # Set subscribers:
            topic_name1 = "/camera1/depth/color/points"
            topic_name2 = "/camera2/depth/color/points"
            camera1 = rospy.Subscriber(topic_name1, PointCloud2, pointcloud_callback_c1)
            camera2 = rospy.Subscriber(topic_name2, PointCloud2, pointcloud_callback_c2)

            # Acquire the scene from both cameras:
            while received_ros_cloud2_c1 is None and not rospy.is_shutdown():
                rospy.loginfo("-- Not receiving ROS PointCloud2 from camera 1 yet ...")
            print "we have received a frame from cam 1"
            Whole_Scene_Cloud_c1 = convert_ros_to_open3d(received_ros_cloud2_c1)
            print(Whole_Scene_Cloud_c1)

            while received_ros_cloud2_c2 is None and not rospy.is_shutdown():
                rospy.loginfo("-- Not receiving ROS PointCloud2 from camera 2 yet ...")
            print "we have received a frame from cam 2"
            Whole_Scene_Cloud_c2 = convert_ros_to_open3d(received_ros_cloud2_c2)
            print(Whole_Scene_Cloud_c2)

            # Write the scenes to specific files for reference
            Whole_Scene_Filename_c1 = PYTHON_FILE_PATH + "Whole_Scene_For_Reconstruction_C1.pcd"
            open3d.io.write_point_cloud(Whole_Scene_Filename_c1, Whole_Scene_Cloud_c1)
            rospy.loginfo("-- Write result point cloud to: " + Whole_Scene_Filename_c1)
            Whole_Scene_Filename_c2 = PYTHON_FILE_PATH + "Whole_Scene_For_Reconstruction_C2.pcd"
            open3d.io.write_point_cloud(Whole_Scene_Filename_c2, Whole_Scene_Cloud_c2)
            rospy.loginfo("-- Write result point cloud to: " + Whole_Scene_Filename_c2)

            source = open3d.io.read_point_cloud("/home/bdml/catkin_ws/src/vision_based_picking/scripts/Whole_Scene_For_Reconstruction_C2.pcd")
            target = open3d.io.read_point_cloud("/home/bdml/catkin_ws/src/vision_based_picking/scripts/Whole_Scene_For_Reconstruction_C1.pcd")

            open3d.visualization.draw_geometries([source])
            source = filter_pc(source, 0.003, [15, 0.01], [30, 0.015])
            open3d.visualization.draw_geometries([target])
            target = filter_pc(target, 0.003, [15, 0.01], [30, 0.015])

            threshold = 0.005
            trans_init = np.asarray([[.23183,0.543548,-0.806728,0.80269],
                                      [-0.514451,0.772362,0.372554,-0.37719],
                                      [0.825587,0.328653,0.458686,0.513103], [0.0, 0.0, 0.0, 1.0]])

            draw_registration_result(source, target, trans_init)
            print("Initial alignment")
            evaluation = open3d.registration.evaluate_registration(source, target, threshold, trans_init)
            print(evaluation)

            print("Apply point-to-point ICP")
            reg_p2p = open3d.registration.registration_icp(
                source, target, threshold, trans_init,
                open3d.registration.TransformationEstimationPointToPoint())
            print(reg_p2p)
            print("Transformation is:")
            print(reg_p2p.transformation)
            print("")
            draw_registration_result(source, target, reg_p2p.transformation)
            Merged_pcl = open3d.geometry.PointCloud.transform(source,reg_p2p.transformation)
            open3d.visualization.draw_geometries([Merged_pcl+target])
            Merged_pcl = Merged_pcl+target
            Merged_pcl = filter_pc(Merged_pcl, 0.003, [15, 0.01], [30, 0.015])
            asd=PointCloud2()
            asd=convertCloudFromOpen3dToRos(Merged_pcl)
            object_to_grasp = open3d.geometry.PointCloud()

            object_to_grasp = segmented_object(asd)
            open3d.visualization.draw_geometries([object_to_grasp])

            object_to_grasp = filter_pc(object_to_grasp, 0.003)
            open3d.visualization.draw_geometries([object_to_grasp])

            my_mesh = open3d.geometry.TriangleMesh()
            my_mesh = object_to_grasp.compute_convex_hull()
            my_mesh.filter_smooth_laplacian()
            my_mesh.filter_sharpen()
            my_mesh.paint_uniform_color([1, 0.706, 0])

            open3d.visualization.draw_geometries([my_mesh])

            CurrentRequest = "Stop"
            have_received_a_cam_frame = 0
        elif CurrentRequest == "Shutdown":
            time.sleep(1)
            exit(0)