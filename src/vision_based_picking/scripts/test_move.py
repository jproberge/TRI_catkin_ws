#!/usr/bin/env python
import sys
import time
import roslib; roslib.load_manifest('ur_driver'); roslib.load_manifest('robotiq_3f_gripper_control')
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from control_msgs.msg import *
from trajectory_msgs.msg import *
from vision_based_picking.srv import Calibrate
from std_msgs.msg import String

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]
Q4 = [1.5567241622891994, -1.729587306997665, 2.0003358809834477, -1.7757690447005094, -1.6760396924665777, 1.690583676669792]

client = None

def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(10.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    g.trajectory.points = [
        JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    client.wait_for_result()
    
def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    
    d = 2.0
    g.trajectory.points = []
    for i in range(10):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    
    client.send_goal(g)
    time.sleep(2.0)
    print "Interrupting"
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def test_move():
    global client, gripper_pub, cam_service
    try:
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("test_move", anonymous=True, disable_signals=True)

        cam_service = rospy.ServiceProxy('handle_calibrate', Calibrate)

        # Connect to ur5
        #client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        #print "Waiting for server..."
        #client.wait_for_server()
        #print "Connected to server"
        
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("manipulator")
      #	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
      #                                                   moveit_msgs.msg.DisplayTrajectory,
      #                                                   queue_size=20)
        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""
        ## END_SUB_TUTORIAL

        current_pose = group.get_current_pose().pose
        print "============ Current pose: %s" % current_pose
        pose_goal = geometry_msgs.msg.Pose()
        #pose_goal.orientation.x = 0.5
        #pose_goal.orientation.y = 0.5
        #pose_goal.orientation.z = -0.5
        #pose_goal.orientation.w = 0.5
        pose_goal.orientation.x = -0.0299808324586
        pose_goal.orientation.y = 0.705266883883
        pose_goal.orientation.z = -0.000660826812059
        pose_goal.orientation.w = 0.708307373597
        pose_goal.position.x = 0.0
        pose_goal.position.y = 0.4012076
        pose_goal.position.z = 0.3159719
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)

        group.stop()

        group.clear_pose_targets()

        current_pose = group.get_current_pose().pose
        print "============ Current pose: %s" % current_pose

        #current_pose = group.get_current_pose().pose
        #print "============ Current pose: %s" % current_pose

        # Initialize robotiq publishers
        #gripper_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output) 
        #command = outputMsg.Robotiq2FGripper_robot_output()
        # command to open gripper
        #command.rPR = 0
        #grippe_pub.publish(command)

        #move1()
        #move_repeated()
        #move_disordered()
        #move_interrupt()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    test_move()
