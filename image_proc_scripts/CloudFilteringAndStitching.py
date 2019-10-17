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

import open3d
import numpy as np
import copy
from ctypes import * # convert float to uint32

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct
import ctypes

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)


def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    # open3d_cloud = open3d.PointCloud()
    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud


def mypointcloud(pointcloud2):
    gen = pc2.read_points(pointcloud2, skip_nans=True)
    int_data = list(gen)
    for x in int_data:
        test = x[3]
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        print r, g, b  # prints r,g,b values in the 0-255 range
                    # x,y,z can be retrieved from the x[0],x[1],x[2]


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
    PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"
    if 0: # test XYZ point cloud format
        filename=PYTHON_FILE_PATH+"test_cloud_XYZ_noRGB.pcd"
    else: # test XYZRGB point cloud format
        filename=PYTHON_FILE_PATH+"test_cloud_XYZRGB.pcd"

    open3d_cloud = open3d.io.read_point_cloud(filename)
    rospy.loginfo("Loading cloud from file by open3d.read_point_cloud: ")
    print(open3d_cloud)
    print("")

    # -- Set publishers
    topic_name1="/camera1/depth/color/points"
    topic_name2="/camera2/depth/color/points"
    #pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1)
    
    # -- Set subscribers
    global received_ros_cloud1
    received_ros_cloud1 = None
    global received_ros_cloud2
    received_ros_cloud2 = None

# Callbacks:
    def pointcloud_callback1(ros_cloud):
        global received_ros_cloud1
        received_ros_cloud1=ros_cloud
        mypointcloud(received_ros_cloud1)
        rospy.loginfo("-- Received ROS PointCloud2 message from camera #1.")

    def pointcloud_callback2(ros_cloud):
        global received_ros_cloud2
        received_ros_cloud2 = ros_cloud
        rospy.loginfo("-- Received ROS PointCloud2 message from camera #2.")

# Assigning the subscribers:
    rospy.Subscriber(topic_name1, PointCloud2, pointcloud_callback1)
    rospy.Subscriber(topic_name2, PointCloud2, pointcloud_callback2)
    
    # -- Convert open3d_cloud to ros_cloud, and publish. Until the subscribe receives it.
    while received_ros_cloud1 is None and not rospy.is_shutdown():
        rospy.loginfo("-- Not receiving ROS PointCloud2 message yet ...")

    while received_ros_cloud2 is None and not rospy.is_shutdown():
        rospy.loginfo("-- Not receiving ROS PointCloud2 message yet ...")
        
    # -- After the reception of a PointCloud2-type point cloud, convert it back to open3d PointCloud() class, and draw  it
    received_open3d_cloud1 = convertCloudFromRosToOpen3d(received_ros_cloud1)
    print(received_open3d_cloud1)
    received_open3d_cloud2 = convertCloudFromRosToOpen3d(received_ros_cloud2)
    print(received_open3d_cloud2)

    # write to file
    output_filename1=PYTHON_FILE_PATH+"PointCloud1.pcd"
    open3d.io.write_point_cloud(output_filename1, received_open3d_cloud1)
    rospy.loginfo("-- Write result point cloud to: "+output_filename1)
    output_filename2=PYTHON_FILE_PATH+"PointCloud2.pcd"
    open3d.io.write_point_cloud(output_filename2, received_open3d_cloud2)
    rospy.loginfo("-- Write result point cloud to: "+output_filename2)

    # draw
    open3d.visualization.draw_geometries([received_open3d_cloud1])
    rospy.loginfo("-- Finish display. The program is terminating ...\n")

    open3d.visualization.draw_geometries([received_open3d_cloud2])
    rospy.loginfo("-- Finish display. The program is terminating ...\n")


'''
This part deals with ICP registration

'''
threshold = 0.02
trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         [-0.139, 0.967, -0.215, 0.7],
                         [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])

