#!/usr/bin/env python

""" ur5_interface.py
    script used to define a simple and easy interface with UR5
    author: Michael Anres Lin (michaelv03@gmail.com)
    date: 10/31/2019
"""

import os
import sys
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np


class UR5Interface:
    """ An interface class for UR5 """

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    joint_values_home = [1.4426736366647526, -1.9116037148601706, 2.0718699175826867, -1.7269285345190166, -1.5725291616141668, 1.442185460127628]

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

    def get_joint_values(self):
        """ get robot joint values """
        return self.group.get_current_joint_values()

    def goto_home_pose(self, wait=True):
        """ go to robot end effector home pose """
        self.goto_joint_target(self.joint_values_home, wait=wait)

    def goto_pose_target(self, pose, wait=True):
        """ go to robot end effector pose target """
        self.group.set_pose_target(pose)
        # simulate in rviz then ask user for feedback
        plan = self.group.plan()
        self.display_trajectory(plan)
        if (wait == True):
            print("============ Press `Enter` to execute the movement ...")
            raw_input()
        self.group.execute(plan, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def goto_joint_target(self, joint_vals, wait=True):
        """ go to robot end effector joint target """
        self.group.set_joint_value_target(joint_vals)
        # simulate in rviz then ask user for feedback
        plan = self.group.plan()
        self.display_trajectory(plan)
        if (wait == True):
            print("============ Press `Enter` to execute the movement ...")
            raw_input()
        self.group.execute(plan, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def stop(self):
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
