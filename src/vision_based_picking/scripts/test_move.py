#!/usr/bin/env python
import sys
import time
import roslib; roslib.load_manifest('ur_driver'); roslib.load_manifest('robotiq_3f_gripper_control')
import rospy
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

        # hard coded home position
        self.pose_home = geometry_msgs.msg.Pose()
        self.pose_home.orientation.x = 0.0
        self.pose_home.orientation.y = 0.7071055
        self.pose_home.orientation.z = 0.0
        self.pose_home.orientation.w = 0.7071055
        self.pose_home.position.x = 0.15
        self.pose_home.position.y = 0.25
        self.pose_home.position.z = 0.25

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
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
        ## END_SUB_TUTORIAL

    def check_joint_limits(self):
        """ function to check that the urdf loaded is specifying
            smaller joint limits (-pi, pi) so that the planner works better """
        for j in joint_names:
            b = self.robot.get_joint(j).bounds()
            # If any joint has limits greater than pi then is bad bounds
            if (b[0] < -(np.pi+0.1)) or (b[1] > (np.pi+0.1)):
                return False

        return True

    def get_pose(self):
        """ get robot end effector pose """
        return self.group.get_current_pose().pose

    def goto_home_pose(self):
        """ go to robot end effector pose target """
        self.group.set_pose_target(self.pose_home)

    def goto_pose_target(self, pose):
        """ go to robot end effector pose target """
        self.group.set_pose_target(pose)
        # simulate in rviz then ask user for feedback


    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

            

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
        if not check_joint_limits(robot):
            raise Exception('Bad joint limits! try running roslaunch with option "limited:=true"')


        current_pose = group.get_current_pose().pose
        print "============ Current pose: %s" % current_pose
        pose_goal = geometry_msgs.msg.Pose()

        ### sampling P1
        pose_goal.orientation.x = 0.0
        pose_goal.orientation.y = 0.7071055
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.7071055
        pose_goal.position.x = 0.0
        pose_goal.position.y = 0.4012076
        pose_goal.position.z = 0.3159719
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)

        group.stop()

        group.clear_pose_targets()

        current_pose = group.get_current_pose().pose
        print "============ P1 pose: %s" % current_pose

        #resp = camera_service('Start', 1)
        #time.sleep(INTER_COMMAND_DELAY)

        #resp = camera_service('Get', 1)
        #time.sleep(INTER_COMMAND_DELAY)

        #camera_service('Shutdown', 1)
    #   resp = CalibrateResponse(0,0,0)
    #   P1 = np.array([[resp.x, resp.y, resp.z]])

    #   ### sampling P2
    #   P2_pose = group.get_current_pose().pose
    #   P2_pose.position.x += 0.1
    #   group.set_pose_target(P2_pose)

    #   plan = group.go(wait=True)

    #   group.stop()

    #   group.clear_pose_targets()

    #   current_pose = group.get_current_pose().pose
    #   print "============ P2 pose: %s" % current_pose

    #   #camera_service('Start', 1)
    #   #time.sleep(INTER_COMMAND_DELAY)

    #   #resp = camera_service('Get', 1)
    #   #time.sleep(INTER_COMMAND_DELAY)

    #   #camera_service('Shutdown', 1)
    #   P2 = np.array([[resp.x, resp.y, resp.z]])

    #   ### sampling P3
    #   P3_pose = group.get_current_pose().pose
    #   P3_pose.position.z += 0.1
    #   group.set_pose_target(P3_pose)

    #   plan = group.go(wait=True)

    #   group.stop()

    #   group.clear_pose_targets()

    #   current_pose = group.get_current_pose().pose
    #   print "============ P3 pose: %s" % current_pose

    #   #camera_service('Start', 1)
    #   #time.sleep(INTER_COMMAND_DELAY)

    #   #resp = camera_service('Get', 1)
    #   #time.sleep(INTER_COMMAND_DELAY)

    #   #camera_service('Shutdown', 1)
    #   P3 = np.array([[resp.x, resp.y, resp.z]])

        print(check_joint_limits(robot))

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
