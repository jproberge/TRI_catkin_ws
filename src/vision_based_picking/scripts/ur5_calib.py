#!/usr/bin/env python

""" ur5_calib.py
    script used to moving the robot to calibrate wrt to realsense camera
    author: Michael Anres Lin (michaelv03@gmail.com)
    date: 10/31/2019
"""

import os
import sys
import time
import roslib; roslib.load_manifest('ur_driver'); roslib.load_manifest('robotiq_3f_gripper_control')
import rospy
from std_msgs.msg import Empty
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from vision_based_picking.srv import Calibrate, CalibrateResponse, Acquire, AcquireResponse
from ur5_interface import UR5Interface
import numpy as np



# global definitions
INTER_COMMAND_DELAY = 4

def sample_led(cam_id):
	# kick of the service package
    os.system("rosrun vision_based_picking FindRobotiqLedCam.py &")
    time.sleep(INTER_COMMAND_DELAY)

    resp = camera_service('Start', cam_id)
    time.sleep(INTER_COMMAND_DELAY)

    resp = camera_service('Get', cam_id)
    time.sleep(INTER_COMMAND_DELAY)

    camera_service('Shutdown', cam_id)
    return np.array([[resp.x, resp.y, resp.z]])

def solve_transform(P1, P2, P3):
    x_hat_C_F = (P2-P1)/np.linalg.norm(P2-P1)
    temp_vec = np.cross((P3-P1), x_hat_C_F)
    y_hat_C_F = temp_vec/np.linalg.norm(temp_vec)
    z_hat_C_F = np.cross(x_hat_C_F, y_hat_C_F)
    T = np.eye(4)
    T[:3,0] = x_hat_C_F
    T[:3,1] = y_hat_C_F
    T[:3,2] = z_hat_C_F
    T[:3,3] = P1
    return T
            

def test_move_home():
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("test_move", anonymous=True, disable_signals=True)

        ur5 = UR5Interface()

        # MoveIt! works well if joint limits are smaller (within -pi, pi)
        if not ur5.check_joint_limits():
            raise Exception('Bad joint limits! try running roslaunch with option "limited:=true"')

        ### go to P1
        ur5.goto_home_pose()
        print(ur5.get_joint_values())


def test_move():
    global client, gripper_pub, camera_service
    try:
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("test_move", anonymous=True, disable_signals=True)

        camera_service = rospy.ServiceProxy('calibrate', Calibrate)

        pcd_service = rospy.ServiceProxy('acquire', Acquire)

        #resp = pcd_service('bowl_pcd.pcd')
        #print("Done")

        ur5 = UR5Interface()

        # MoveIt! works well if joint limits are smaller (within -pi, pi)
        if not ur5.check_joint_limits():
            raise Exception('Bad joint limits! try running roslaunch with option "limited:=true"')


        current_pose = ur5.get_pose()
        print "============ Current pose: %s" % current_pose
        

        ### go to P1
        ur5.goto_home_pose()

        ### sampling P1
        P1_C1 = sample_led(1)
        P1_C2 = sample_led(2)

        ### go to P2
        P2_pose = ur5.get_pose()
        P2_pose.position.x += 0.1
        ur5.goto_pose_target(P2_pose)

        ### sampling P2
        P2_C1 = sample_led(1)
        P2_C2 = sample_led(2)

        ### go to P3
        P3_pose = ur5.get_pose()
        P3_pose.position.z += 0.1
        ur5.goto_pose_target(P3_pose)

        ### sampling P3
        P3_C1 = sample_led(1)
        P3_C2 = sample_led(2)

        print("Camera1 P1: %s, \nP2: %s, \nP3: %s" % (P1_C1, P2_C1 ,P3_C1))
        # Get the transform from Camera to calibration coordinates F
        T_C1_F = solve_transform(P1_C1, P2_C1, P3_C1)

        print("Camera2 P1: %s, \nP2: %s, \nP3: %s" % (P1_C2, P2_C2 ,P3_C2))
        # Get the transform from Camera to calibration coordinates F
        T_C2_F = solve_transform(P1_C2, P2_C2, P3_C2)
        
        # Transform from robot end effector to F
        T_F_6 = np.eye(4)
        T_F_6[:3,3] = np.array([0.0, 0.0375, -0.0115])

        T_O_6 = np.eye(4)
        T_O_6[:3,3] = np.array([-0.15,0.35,0.25])


        T_C1_6 = T_C1_F.dot(T_F_6)
        T_C1_O = T_C1_6.dot(np.linalg.inv(T_O_6))
        T_O_C1 = np.linalg.inv(T_C1_O)
        path = os.path.dirname(os.path.abspath(__file__)) + "/config/"
        np.save(path + 'T_O_C1.npy', T_O_C1)

        T_C2_6 = T_C2_F.dot(T_F_6)
        T_C2_O = T_C2_6.dot(np.linalg.inv(T_O_6))
        T_O_C2 = np.linalg.inv(T_C1_O)
        np.save(path + 'T_O_C2.npy', T_O_C2)

        np.save(path + 'T_C2_C1.npy', np.linalg.inv(T_O_C2).dot(T_O_C1))

        #current_pose = group.get_current_pose().pose
        #print "============ Current pose: %s" % current_pose

        # Initialize robotiq publishers
        #gripper_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output) 
        #command = outputMsg.Robotiq2FGripper_robot_output()
        # command to open gripper
        #command.rPR = 0
        #grippe_pub.publish(command)

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


# Change this macro to True if needed to re-calibrate
PERFORM_CALIBRATION = True

if __name__ == '__main__': 

    if PERFORM_CALIBRATION:
        # uncomment test move to perform a calibration
        test_move()
    else:
        P1 = np.array([ 0.11121763, 0.13853986, 0.80320004])
        P2 = np.array([ 0.0433853,  0.09843211, 0.85500003])
        P3 = np.array([ 0.05640479, 0.01420273, 0.81033337])

        T_C_F = solve_transform(P1, P2, P3)
        
        # Transform from robot end effector to F
        T_F_6 = np.eye(4)
        T_F_6[:3,3] = np.array([0.0, 0.0375, -0.0115])

        T_C_6 = T_C_F.dot(T_F_6)

        T_O_6 = np.eye(4)
        T_O_6[:3,3] = np.array([-0.15,0.35,0.25])

        T_C_O = T_C_6.dot(np.linalg.inv(T_O_6))
        
        np.save('T_O_C1.npy', np.linalg.inv(T_C_O))

    path = os.path.join(os.path.dirname(__file__)) + "/config/"
    T_O_C1 = np.load(path + 'T_O_C1.npy')
    T_O_C2 = np.load(path + 'T_O_C2.npy')
    np.save(path + 'T_C2_C1.npy', np.linalg.inv(T_O_C2).dot(T_O_C1))
    print(T_O_C1)
