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
from vision_based_picking.srv import Calibrate, CalibrateResponse

import numpy as np



# global definitions
INTER_COMMAND_DELAY = 8


class UR5Interface:
    """ An interface class for UR5 """

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

        self.goal_state_publisher = rospy.Publisher('/rviz/moveit/update_custom_goal_state',
                                                        moveit_msgs.msg.RobotState,
                                                        queue_size=20)

        # hard coded home position
        self.pose_home = geometry_msgs.msg.Pose()
        self.pose_home.orientation.x = 0.0
        self.pose_home.orientation.y = 0.7071055
        self.pose_home.orientation.z = 0.0
        self.pose_home.orientation.w = 0.7071055
        self.pose_home.position.x = -0.15
        self.pose_home.position.y = 0.35
        self.pose_home.position.z = 0.25

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print "============ Robot Groups:", self.robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""

        self.group.set_max_acceleration_scaling_factor(0.1)
        self.group.set_max_velocity_scaling_factor(0.1)
        print "============ Set a max acceleration value of 0.1"
        print "============ Set a max velocity value of 0.1"

    def check_joint_limits(self):
        """ function to check that the urdf loaded is specifying
            smaller joint limits (-pi, pi) so that the planner works better """
        for j in self.joint_names:
            b = self.robot.get_joint(j).bounds()
            # If any joint has limits greater than pi then is bad bounds
            if (b[0] < -(np.pi+0.1)) or (b[1] > (np.pi+0.1)):
                return False

        return True

    def get_pose(self):
        """ get robot end effector pose """
        return self.group.get_current_pose().pose

    def goto_home_pose(self):
        """ go to robot end effector home pose """
        self.goto_pose_target(self.pose_home)

    def goto_pose_target(self, pose):
        """ go to robot end effector pose target """
        self.group.set_pose_target(pose)
        # simulate in rviz then ask user for feedback
        plan = self.group.plan()
        #self.display_trajectory(plan)
        print("============ Press `Enter` to execute the movement ...")
        raw_input()
        self.group.execute(plan, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()


    def display_trajectory(self, plan):
        """ displays planned trajectory in rviz """
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
        # not working for goal state yet
        robot_goal_state = moveit_msgs.msg.RobotState()
        robot_goal_state.joint_state.position = plan.joint_trajectory.points[-1].positions
        self.goal_state_publisher.publish(robot_goal_state)

def sample_led():
	# kick of the service package
	os.system("rosrun vision_based_picking FindRobotiqLedCam.py &")
        time.sleep(INTER_COMMAND_DELAY)

        resp = camera_service('Start', 1)
        time.sleep(INTER_COMMAND_DELAY)

        resp = camera_service('Get', 1)
        time.sleep(INTER_COMMAND_DELAY)

        camera_service('Shutdown', 1)
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
            

def test_move():
    global client, gripper_pub, camera_service
    try:
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("test_move", anonymous=True, disable_signals=True)

        camera_service = rospy.ServiceProxy('calibrate', Calibrate)

        #robot = moveit_commander.RobotCommander()
        #scene = moveit_commander.PlanningSceneInterface()
        #group = moveit_commander.MoveGroupCommander("manipulator")
      #	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
      #                                                   moveit_msgs.msg.DisplayTrajectory,
      #                                                   queue_size=20)
        ur5 = UR5Interface()

        # MoveIt! works well if joint limits are smaller (within -pi, pi)
        if not ur5.check_joint_limits():
            raise Exception('Bad joint limits! try running roslaunch with option "limited:=true"')


        current_pose = ur5.get_pose()
        print "============ Current pose: %s" % current_pose
        

        ### go to P1
        ur5.goto_home_pose()

        ### sampling P1
        P1 = sample_led()

        ### go to P2
        P2_pose = ur5.get_pose()
        P2_pose.position.x += 0.1
        ur5.goto_pose_target(P2_pose)

        ### sampling P2
        P2 = sample_led()

        ### go to P3
        P3_pose = ur5.get_pose()
        P3_pose.position.z += 0.1
        ur5.goto_pose_target(P3_pose)

        ### sampling P3
        P3 = sample_led()

        print("P1: %s, \nP2: %s, \nP3: %s" % (P1, P2 ,P3))
        # Get the transform from Camera to calibration coordinates F
        T_C_F = solve_transform(P1, P2, P3)
        
        # Transform from robot end effector to F
        T_F_6 = np.eye(4)
        T_F_6[:3,3] = np.array([0.0, 0.0375, -0.0115])

        T_C_6 = T_C_F.dot(T_F_6)
        T_C_6 = T_C_F.dot(T_F_6)

        T_O_6 = np.eye(4)
        T_O_6[:3,3] = np.array([-0.15,0.35,0.25])

        T_C_O = T_C_6.dot(np.linalg.inv(T_O_6))
        
        np.save('T_R_C.npy', T_C_O)


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

if __name__ == '__main__': 
    test_move()
    #P1 = np.array([[0.4, 0.4, 0.0]])
    #P2 = np.array([[0.4, -0.4, 0.0]])
    #P3 = np.array([[0.0, 0.0, 1.0]])
    #T = solve_transform(P1,P2,P3)
    #np.save('T_R_C.npy', T)

    T = np.load('T_R_C.npy')
    print(T)
