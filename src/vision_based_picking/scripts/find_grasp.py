#!/usr/bin/env python

from ur5_interface import UR5Interface
from robotiq_interface import RobotiqInterface
import open3d as o3d
import numpy as np
import copy
import time
import os

import rospy
import tf
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial.transform import Rotation as R
from dbot_ros_msgs.msg import ObjectState

bowl_pose = None
ur5_pose = None
got_bowl_pose = False
got_ur5_pose = False
got_all_pose = False

def get_object_pose_cb(data):
    global bowl_pose
    global got_bowl_pose
    global ur5_pose
    global got_ur5_pose
    global got_all_pose
    if (data.name == "bowl_meter"):
        bowl_pose = data.pose.pose
        got_bowl_pose = True
    if (data.name == "ur5Wrist_meter"):
        ur5_pose = data.pose.pose
        got_ur5_pose = True
    if (got_ur5_pose and got_bowl_pose):
        got_ur5_pose = False
        got_bowl_pose = False
        got_all_pose = True

def servo_to_obj():
    global bowl_pose
    global got_bowl_pose
    global ur5_pose
    global got_ur5_pose
    global got_all_pose
    try:
        path = os.path.dirname(os.path.abspath(__file__)) + "/config/"
        T_O_C1 = np.load(path + 'T_O_C1.npy')
        T_O_C2 = np.load(path + 'T_O_C2.npy')

        rospy.init_node("find_grasp", anonymous=True, disable_signals=True)
        rospy.Subscriber('/particle_tracker/object_state', ObjectState, get_object_pose_cb)

        ur5 = UR5Interface()

        gripper = RobotiqInterface()

        # Move the UR5 to a home position
        ur5.goto_home_pose()

        gripper.goto_gripper_pos(0)


        while (not got_all_pose):
            time.sleep(0.1)
        got_all_pose = False

        target_pos = np.zeros(3)
        target_pos[0] = bowl_pose.position.x
        target_pos[1] = bowl_pose.position.y
        target_pos[2] = bowl_pose.position.z
        
        target_rot = np.zeros(4)
        target_rot[0] = bowl_pose.orientation.x
        target_rot[1] = bowl_pose.orientation.y
        target_rot[2] = bowl_pose.orientation.z
        target_rot[3] = bowl_pose.orientation.w

        T_A_C = np.zeros((4,4))
        T_A_C[3,3] = 1.0
        T_A_C[:3,3] = target_pos
        T_A_C[:3,:3] = R.from_quat(target_rot).as_dcm()
        

        print(T_A_C)
        # we want to find now T_O_A
        T_O_A = T_O_C1.dot(np.linalg.inv(T_A_C))


        # MoveIt! works well if joint limits are smaller (within -pi, pi)
        #if not ur5.check_joint_limits():
        #    raise Exception('Bad joint limits! try running roslaunch with option "limited:=true"')


        br = tf.TransformBroadcaster()

        while (not rospy.is_shutdown()):
            time.sleep(1)
            print("============ Next Grasp Press Enter  ...")
            raw_input()

            r1 = R.from_dcm(T_O_C1[:3,:3])
            br.sendTransform(T_O_C1[:3,3],
                             r1.as_quat(),
                             rospy.Time.now(),
                             "cam1",
                             "base_link")

            while (not got_all_pose):
                time.sleep(0.1)
            got_all_pose = False

            target_pos = np.zeros(3)
            target_pos[0] = bowl_pose.position.x - ur5_pose.position.x
            target_pos[1] = bowl_pose.position.y - ur5_pose.position.y
            target_pos[2] = bowl_pose.position.z - ur5_pose.position.z
            
            target_rot = np.zeros(4)
            target_rot[0] = bowl_pose.orientation.x
            target_rot[1] = bowl_pose.orientation.y
            target_rot[2] = bowl_pose.orientation.z
            target_rot[3] = bowl_pose.orientation.w

            
            print("relative pose :\n" + str(target_pos))
            # we want to find now p_O_A so we rotate by calibration matrix
            p_O_A = T_O_C1[:3,:3].dot(target_pos)
            
            print("relative pose rotated :\n" + str(p_O_A))


            #r3 = R.from_dcm(T_O_A[:3,:3])
            #br.sendTransform(T_O_A[:3,3],
            #                  r3.as_quat(),
            #                  rospy.Time.now(),
            #                  "obj",
            #                  "base_link")

            # get gripper rotation
            target_pose = ur5.get_pose()
            target_pose.position.x += p_O_A[0]
            target_pose.position.y += p_O_A[1]
            target_pose.position.z += p_O_A[2] + 0.3
            ur5.goto_pose_target(target_pose)

            target_pose.position.x += 0.11
            ur5.goto_pose_target(target_pose)

            target_pose.position.z -= 0.07
            ur5.goto_pose_target(target_pose)
            gripper.goto_gripper_pos(240)

            target_pose.position.z += 0.2
            ur5.goto_pose_target(target_pose)
            gripper.goto_gripper_pos(0)
    except KeyboardInterrupt:
        ur5.stop()
        print("UR stopped")

if __name__ == "__main__":
    servo_to_obj()
