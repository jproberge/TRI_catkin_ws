#!/usr/bin/env python

from ur5_interface import UR5Interface
import open3d as o3d
import numpy as np
import copy
import time
import os

import rospy
import tf
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R
from dbot_ros_msgs.msg import ObjectState

curr_pose = None
got_curr_pose = False

def get_object_pose_cb(data):
    global curr_pose
    global got_curr_pose
    curr_pose = data.pose.pose
    got_curr_pose = True

def servo_to_obj():
    global curr_pose
    global got_curr_pose
    try:
        path = os.path.dirname(os.path.abspath(__file__)) + "/config/"
        T_O_C1 = np.load(path + 'T_O_C1.npy')
        T_O_C2 = np.load(path + 'T_O_C2.npy')

        rospy.init_node("find_grasp", anonymous=True, disable_signals=True)
        rospy.Subscriber('/object_tracker_service/object_state', ObjectState, get_object_pose_cb)

        ur5 = UR5Interface()

        while (not got_curr_pose):
            time.sleep(0.1)

        curr_pos = np.zeros(3)
        curr_pos[0] = curr_pose.position.x
        curr_pos[1] = curr_pose.position.y
        curr_pos[2] = curr_pose.position.z
        
        curr_rot = np.zeros(4)
        curr_rot[0] = curr_pose.orientation.x
        curr_rot[1] = curr_pose.orientation.y
        curr_rot[2] = curr_pose.orientation.z
        curr_rot[3] = curr_pose.orientation.w

        T_A_C = np.zeros((4,4))
        T_A_C[3,3] = 1.0
        T_A_C[:3,3] = curr_pos
        T_A_C[:3,:3] = R.from_quat(curr_rot).as_dcm()
        

        print(T_A_C)
        # we want to find now T_O_A
        T_O_A = T_O_C1.dot(np.linalg.inv(T_A_C))

        # get gripper rotation
        target_pose = ur5.get_pose()
        target_pose.position.x = T_O_A[0,3]
        target_pose.position.y = T_O_A[1,3]
        target_pose.position.z = T_O_A[2,3] + 0.3

        # MoveIt! works well if joint limits are smaller (within -pi, pi)
        if not ur5.check_joint_limits():
            raise Exception('Bad joint limits! try running roslaunch with option "limited:=true"')

        #ur5.goto_pose_target(target_pose)

        br = tf.TransformBroadcaster()

        while (not rospy.is_shutdown()):
            time.sleep(1)

            r1 = R.from_dcm(T_O_C1[:3,:3])
            br.sendTransform(T_O_C1[:3,3],
                             r1.as_quat(),
                             rospy.Time.now(),
                             "cam1",
                             "base_link")

            r3 = R.from_dcm(T_O_A[:3,:3])
            br.sendTransform(T_O_A[:3,3],
                              r3.as_quat(),
                              rospy.Time.now(),
                              "obj",
                              "base_link")
    except KeyboardInterrupt:
        ur5.stop()
        print("UR stopped")

if __name__ == "__main__":
    servo_to_obj()
