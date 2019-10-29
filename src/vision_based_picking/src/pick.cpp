/*
 * File: pick.cpp
 * Description: A simple file provided as an example to show hot to calibrate, to process vision data, and to pick an object based on
 * the segmented object's location. The pipeline is roughly: 1) Move the robot to its initial position (home position is set such that
 * it does not interfere with the cameras' point of view), 2) Acquire visual inputs 3) perform an automatic extrinsic calibration procedure
 * to determine the camera poses with reference to the robot's base reference frame, 4) based on calibration, reconstruct the scene with
 * the two cameras + ICP, 5) segment the object of interest in that scene and 5) pick the object. The object picking is only a naive picking
 * attempt for now, more work should be done to develop a proper grasp planing strategy.
 *
 * Author: Jean-Philippe Roberge (jean-philippe.roberge@etsmtl.ca)
 * Date: October 13 2019
*/

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <boost/thread/thread.hpp>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
//#include "vision_based_picking/Calibrate.h"
#include "roboticArm.h"
#include "URcontrolV2.h"
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <ur_msgs/FollowCartesianTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

#include "calibration_utils.h"

#define INTER_COMMAND_DELAY 8


// Used for defining trajectory (i.e. multiple target poses to reach from a single command).
typedef ur_msgs::FollowCartesianTrajectoryGoal CartesianTrajectory;

// Using boost (for multithreading) and std namespaces
using namespace std;
using namespace boost::posix_time;

// Function prototypes:
vector<double> cross_product(vector<double> V1, vector<double> V2);
void ExportMatrixToCSV(tf::Transform Matrix, string filename);
bool cmdOptionExists(char** begin, char** end, const string& option);
char* getCmdOption(char ** begin, char ** end, const string & option);


//  cout << "The pose is: " << APoseToReach1.getBasis().getRow(0).x() << " " << APoseToReach1.getBasis().getRow(0).y() << " " << APoseToReach1.getBasis().getRow(0).z() << " " << APoseToReach1.getBasis().getRow(0).w() << endl;
//  cout << "The pose is: " << APoseToReach1.getBasis().getRow(1).x() << " " << APoseToReach1.getBasis().getRow(1).y() << " " << APoseToReach1.getBasis().getRow(1).z() << " " << APoseToReach1.getBasis().getRow(1).w() << endl;
//  cout << "The pose is: " << APoseToReach1.getBasis().getRow(2).x() << " " << APoseToReach1.getBasis().getRow(2).y() << " " << APoseToReach1.getBasis().getRow(2).z() << " " << APoseToReach1.getBasis().getRow(2).w() << endl;
//  cout << "The pose is: " << APoseToReach1.getBasis().getRow(3).x() << " " << APoseToReach1.getBasis().getRow(3).y() << " " << APoseToReach1.getBasis().getRow(3).z() << " " << APoseToReach1.getBasis().getRow(3).w() << endl;

//  cout << "The pose is: " << APoseToReach1.getBasis().getColumn(0).x() << " " << APoseToReach1.getBasis().getColumn(0).y() << " " << APoseToReach1.getBasis().getColumn(0).z() << " " << APoseToReach1.getBasis().getColumn(0).w() << endl;
//  cout << "The pose is: " << APoseToReach1.getBasis().getColumn(1).x() << " " << APoseToReach1.getBasis().getColumn(1).y() << " " << APoseToReach1.getBasis().getColumn(1).z() << " " << APoseToReach1.getBasis().getColumn(1).w() << endl;
//  cout << "The pose is: " << APoseToReach1.getBasis().getColumn(2).x() << " " << APoseToReach1.getBasis().getColumn(2).y() << " " << APoseToReach1.getBasis().getColumn(2).z() << " " << APoseToReach1.getBasis().getColumn(2).w() << endl;
//  cout << "The pose is: " << APoseToReach1.getBasis().getColumn(3).x() << " " << APoseToReach1.getBasis().getColumn(3).y() << " " << APoseToReach1.getBasis().getColumn(3).z() << " " << APoseToReach1.getBasis().getColumn(3).w() << endl;


int main(int argc, char **argv)
{
  // Initializatinos:
  ros::init(argc, argv, "pick");
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandle n2;
  ros::ServiceClient srv_client = n2.serviceClient<vision_based_picking::Calibrate>("calibrate");
  ros::Rate loop_rate(1000); // desired loop frequency (this will change how fast we try to check if new messages have arrived)

  vision_based_picking::Calibrate srv;
  RoboticArm robot; //The main robot control object
  Client client ("follow_cart_trajectory");
  client.waitForServer();

  tf::Transform tfpose_init, tfpose_oriented, CalibrationPoseC1_P1, CalibrationPoseC1_P2, CalibrationPoseC1_P3, CalibrationPoseC2_P1, CalibrationPoseC2_P2, CalibrationPoseC2_P3,O_T_C1,O_T_C2,C1_T_F,C2_T_F,F_T_6,O_T_6,C1_T_C2,C2_T_C1,TmpPose1,TmpPose2; //Defining some poses instances
  tf::Vector3 ASetOfCoordinnates; // Translation/Position vectors
  static tf::TransformBroadcaster br1, br2, br3;

  double P1x, P1y, P1z, P2x, P2y, P2z, P3x, P3y, P3z, eucl_dist;
  vector<double> C_x_F(3), C_y_F(3), C_z_F(3), V_cross_prod(3), My_Vec(3);
  bool PerformCalibration=false;

  if(cmdOptionExists(argv, argv+argc, "-CALIBRATE"))
  {
    PerformCalibration=true;
  }
  else
  {
    PerformCalibration=false;
  }


  /**************************************************************************/
  /************************* CALIBRATION PROCEDURE **************************/
  /**************************************************************************/
  if (PerformCalibration == false)
  {
    ifstream file1 ( "/home/bdml/catkin_ws/CalibrationMatrices/O_T_C1.csv" );
    ifstream file2 ( "/home/bdml/catkin_ws/CalibrationMatrices/O_T_C2.csv" );
    string value;
    char delim = ',';
    tf::Matrix3x3 TmpRotation;
    tf::Vector3 TmpPosition;
    int line_index=1;
    double r11,r12,r13,t1;
    double r21,r22,r23,t2;
    double r31,r32,r33,t3;
    std::string field;
    vector<string> elements;
    string tmp;

    cout << endl << endl << "Calibration matrices were automatically loaded: " << endl << endl;
    // Parse first file to fill the first calibration matrix: (O_T_C1)
    while (getline(file1,value))
    {
      if (line_index==1)
      {
        std::stringstream ss(value);
        while (getline(ss,tmp,delim)) {
          elements.push_back(tmp);
        }
        r11=atof(elements.at(0).c_str());
        r12=atof(elements.at(1).c_str());
        r13=atof(elements.at(2).c_str());
        t1=atof(elements.at(3).c_str());
        elements.clear();
      }
      else if (line_index==2)
      {
        std::stringstream ss(value);
        while (getline(ss,tmp,delim)) {
          elements.push_back(tmp);
        }
        r21=atof(elements.at(0).c_str());
        r22=atof(elements.at(1).c_str());
        r23=atof(elements.at(2).c_str());
        t2=atof(elements.at(3).c_str());
        elements.clear();
      }
      else if (line_index==3)
      {
        std::stringstream ss(value);
        while (getline(ss,tmp,delim)) {
          // Provide proper checks here for tmp like if empty
          // Also strip down symbols like !, ., ?, etc.
          // Finally push it.
          elements.push_back(tmp);
        }
        r31=atof(elements.at(0).c_str());
        r32=atof(elements.at(1).c_str());
        r33=atof(elements.at(2).c_str());
        t3=atof(elements.at(3).c_str());
        elements.clear();
      }
      // read a string until next comma: http://www.cplusplus.com/reference/string/getline/
      line_index++;
    }
    TmpRotation.setValue(r11,r21,r31,r12,r22,r32,r13,r23,r33);
    TmpPosition.setValue(t1,t2,t3);
    O_T_C1.setBasis(TmpRotation);
    O_T_C1.setOrigin(TmpPosition);

    cout << "O_T_C1: " << endl;
    cout << O_T_C1.getBasis().getRow(0).x() << " " << O_T_C1.getBasis().getRow(1).x() << " " << O_T_C1.getBasis().getRow(2).x() << " " << O_T_C1.getOrigin().getX() << endl;
    cout << O_T_C1.getBasis().getRow(0).y() << " " << O_T_C1.getBasis().getRow(1).y() << " " << O_T_C1.getBasis().getRow(2).y() << " " << O_T_C1.getOrigin().getY() << endl;
    cout << O_T_C1.getBasis().getRow(0).z() << " " << O_T_C1.getBasis().getRow(1).z() << " " << O_T_C1.getBasis().getRow(2).z() << " " << O_T_C1.getOrigin().getZ() << endl;
    cout << endl << endl;
    line_index=1;

    // Parse first file to fill the first calibration matrix: (O_T_C1)
    while (getline(file2,value))
    {
      if (line_index==1)
      {
        std::stringstream ss(value);
        while (getline(ss,tmp,delim)) {
          elements.push_back(tmp);
        }
        r11=atof(elements.at(0).c_str());
        r12=atof(elements.at(1).c_str());
        r13=atof(elements.at(2).c_str());
        t1=atof(elements.at(3).c_str());
        elements.clear();
      }
      else if (line_index==2)
      {
        std::stringstream ss(value);
        while (getline(ss,tmp,delim)) {
          elements.push_back(tmp);
        }
        r21=atof(elements.at(0).c_str());
        r22=atof(elements.at(1).c_str());
        r23=atof(elements.at(2).c_str());
        t2=atof(elements.at(3).c_str());
        elements.clear();
      }
      else if (line_index==3)
      {
        std::stringstream ss(value);
        while (getline(ss,tmp,delim)) {
          elements.push_back(tmp);
        }
        r31=atof(elements.at(0).c_str());
        r32=atof(elements.at(1).c_str());
        r33=atof(elements.at(2).c_str());
        t3=atof(elements.at(3).c_str());
        elements.clear();
      }
      line_index++;
    }
    TmpRotation.setValue(r11,r21,r31,r12,r22,r32,r13,r23,r33);
    TmpPosition.setValue(t1,t2,t3);
    O_T_C2.setBasis(TmpRotation);
    O_T_C2.setOrigin(TmpPosition);
    cout << "O_T_C2: " << endl;
    cout << O_T_C2.getBasis().getRow(0).x() << " " << O_T_C2.getBasis().getRow(1).x() << " " << O_T_C2.getBasis().getRow(2).x() << " " << O_T_C2.getOrigin().getX() << endl;
    cout << O_T_C2.getBasis().getRow(0).y() << " " << O_T_C2.getBasis().getRow(1).y() << " " << O_T_C2.getBasis().getRow(2).y() << " " << O_T_C2.getOrigin().getY() << endl;
    cout << O_T_C2.getBasis().getRow(0).z() << " " << O_T_C2.getBasis().getRow(1).z() << " " << O_T_C2.getBasis().getRow(2).z() << " " << O_T_C2.getOrigin().getZ() << endl;
    cout << endl << endl;

    C1_T_C2.mult(O_T_C1.inverse(),O_T_C2);
    C2_T_C1.mult(O_T_C2.inverse(),O_T_C1);

    cout << "C1_T_C2: " << endl;
    cout << C1_T_C2.getBasis().getRow(0).x() << " " << C1_T_C2.getBasis().getRow(1).x() << " " << C1_T_C2.getBasis().getRow(2).x() << " " << C1_T_C2.getOrigin().getX() << endl;
    cout << C1_T_C2.getBasis().getRow(0).y() << " " << C1_T_C2.getBasis().getRow(1).y() << " " << C1_T_C2.getBasis().getRow(2).y() << " " << C1_T_C2.getOrigin().getY() << endl;
    cout << C1_T_C2.getBasis().getRow(0).z() << " " << C1_T_C2.getBasis().getRow(1).z() << " " << C1_T_C2.getBasis().getRow(2).z() << " " << C1_T_C2.getOrigin().getZ() << endl;
    cout << endl << endl;

    cout << "C2_T_C1: " << endl;
    cout << C2_T_C1.getBasis().getRow(0).x() << " " << C2_T_C1.getBasis().getRow(1).x() << " " << C2_T_C1.getBasis().getRow(2).x() << " " << C2_T_C1.getOrigin().getX() << endl;
    cout << C2_T_C1.getBasis().getRow(0).y() << " " << C2_T_C1.getBasis().getRow(1).y() << " " << C2_T_C1.getBasis().getRow(2).y() << " " << C2_T_C1.getOrigin().getY() << endl;
    cout << C2_T_C1.getBasis().getRow(0).z() << " " << C2_T_C1.getBasis().getRow(1).z() << " " << C2_T_C1.getBasis().getRow(2).z() << " " << C2_T_C1.getOrigin().getZ() << endl;
    cout << endl << endl;
  }
  else
  {
    // Showing the P1 to camera #1:
    tfpose_init = robot.get_arm_position("base_link");
    orientEffectorTowardsTable(tfpose_init, tfpose_oriented); //Calculate matrix to orient gripper orthogonally with reference to the table
    ASetOfCoordinnates.setValue(0.15,0.25,0.25); //Home Position
    CalibrationPoseC1_P1.setBasis(tfpose_oriented.getBasis());
    CalibrationPoseC1_P1.setOrigin(ASetOfCoordinnates);
    robot.move_to_point(CalibrationPoseC1_P1,"base_link",0.05,client); //move to the home position to prevent visual occlusion
    client.waitForResult();

    // Starting the vision/LED segmentation process:
    int res = system("rosrun vision_based_picking FindRobotiqLedCam.py &");
    cout << "The system call exit code was: " << res << endl;
    sleep(INTER_COMMAND_DELAY); // give the vision process a bit of time to start
    srv.request.cam_index=1;
    srv.request.the_request="Start";

    if (srv_client.call(srv))
    {
      ROS_INFO("The Start request was sent to the vision server");
    }
    sleep(INTER_COMMAND_DELAY); // give the vision process a bit of time to start
    srv.request.the_request="Get";
    if (srv_client.call(srv))
    {
      ROS_INFO("Robotiq Led in Camera #1 frame: {%f, %f, %f,}",srv.response.x, srv.response.y, srv.response.z);
      P1x=srv.response.x; P1y=srv.response.y; P1z=srv.response.z;
    }
    else
    {
      P1x=0; P1y=0; P1z=0;
    }
    sleep(INTER_COMMAND_DELAY); // give the vision process a bit of time to start
    srv.request.the_request="Shutdown";
    srv.request.cam_index=1;
    if (srv_client.call(srv))
    {
      ROS_INFO("The Shutdown request was sent to the vision server");
    }


    // Showing the P2 to camera #1:
    ASetOfCoordinnates=CalibrationPoseC1_P1.getOrigin();
    ASetOfCoordinnates.setX(CalibrationPoseC1_P1.getOrigin().getX()+0.1);
    CalibrationPoseC1_P2.setOrigin(ASetOfCoordinnates);
    CalibrationPoseC1_P2.setBasis(CalibrationPoseC1_P1.getBasis());
    robot.move_to_point(CalibrationPoseC1_P2,"base_link",0.05,client); //move to the home position to prevent visual occlusion
    client.waitForResult();

    // Starting the vision/LED segmentation process:
    res = system("rosrun vision_based_picking FindRobotiqLedCam.py &");
    cout << "The system call exit code was: " << res << endl;
    sleep(INTER_COMMAND_DELAY); // give the vision process a bit of time to start
    srv.request.cam_index=1;
    srv.request.the_request="Start";

    if (srv_client.call(srv))
    {
      ROS_INFO("The Start request was sent to the vision server");
    }
    sleep(INTER_COMMAND_DELAY); // give the vision process a bit of time to start
    srv.request.the_request="Get";
    if (srv_client.call(srv))
    {
      ROS_INFO("Robotiq Led in Camera #1 frame: {%f, %f, %f,}",srv.response.x, srv.response.y, srv.response.z);
      P2x=srv.response.x; P2y=srv.response.y; P2z=srv.response.z;
    }
    else
    {
      P2x=0; P2y=0; P2z=0;
    }
    sleep(INTER_COMMAND_DELAY); // give the vision process a bit of time to start
    srv.request.the_request="Shutdown";
    srv.request.cam_index=1;
    if (srv_client.call(srv))
    {
      ROS_INFO("The Shutdown request was sent to the vision server");
    }

    // Showing the P3 to camera #1:
    ASetOfCoordinnates=CalibrationPoseC1_P1.getOrigin();
    ASetOfCoordinnates.setZ(CalibrationPoseC1_P1.getOrigin().getZ()+0.1);
    CalibrationPoseC1_P3.setOrigin(ASetOfCoordinnates);
    CalibrationPoseC1_P3.setBasis(CalibrationPoseC1_P1.getBasis());
    robot.move_to_point(CalibrationPoseC1_P3,"base_link",0.05,client); //move to the home position to prevent visual occlusion
    client.waitForResult();

    // Starting the vision/LED segmentation process:
    res = system("rosrun vision_based_picking FindRobotiqLedCam.py &");
    cout << "The system call exit code was: " << res << endl;
    sleep(INTER_COMMAND_DELAY); // give the vision process a bit of time to start
    srv.request.cam_index=1;
    srv.request.the_request="Start";

    if (srv_client.call(srv))
    {
      ROS_INFO("The Start request was sent to the vision server");
    }
    sleep(INTER_COMMAND_DELAY); // give the vision process a bit of time to start
    srv.request.the_request="Get";
    if (srv_client.call(srv))
    {
      ROS_INFO("Robotiq Led in Camera #1 frame: {%f, %f, %f,}",srv.response.x, srv.response.y, srv.response.z);
      P3x=srv.response.x; P3y=srv.response.y; P3z=srv.response.z;
    }
    else
    {
      P3x=0; P3y=0; P3z=0;
    }
    sleep(INTER_COMMAND_DELAY); // give the vision process a bit of time to start
    srv.request.the_request="Shutdown";
    srv.request.cam_index=1;
    if (srv_client.call(srv))
    {
      ROS_INFO("The Shutdown request was sent to the vision server");
    }


    // Solving camera #1's extrinsic calibration:
    eucl_dist=sqrt(pow(P2x-P1x,2)+pow(P2y-P1y,2)+pow(P2z-P1z,2));
    C_x_F[0]=(P2x-P1x)/eucl_dist; // The feature's x-axis, as seen in the camera's reference frame
    C_x_F[1]=(P2y-P1y)/eucl_dist;
    C_x_F[2]=(P2z-P1z)/eucl_dist;
    cout << "The Euclidean distance is: " << eucl_dist << "The x unit vector is: [" << C_x_F[0] << "," <<C_x_F[1] <<"," << C_x_F[2] <<  endl;
    My_Vec[0]=P3x-P1x;
    My_Vec[1]=P3y-P1y;
    My_Vec[2]=P3z-P1z;
    V_cross_prod=cross_product(C_x_F,My_Vec);
    eucl_dist=sqrt(pow(V_cross_prod[0],2)+pow(V_cross_prod[1],2)+pow(V_cross_prod[2],2));
    C_y_F[0]=V_cross_prod[0]/eucl_dist;
    C_y_F[1]=V_cross_prod[1]/eucl_dist;
    C_y_F[2]=V_cross_prod[2]/eucl_dist;
    cout << "The Euclidean distance is: " << eucl_dist << "The y unit vector is: [" << C_y_F[0] << "," <<C_y_F[1] <<"," << C_y_F[2] <<  endl;
    C_z_F=cross_product(C_x_F,C_y_F);
    cout << "The Euclidean distance is: " << eucl_dist << "The z unit vector is: [" << C_z_F[0] << "," <<C_z_F[1] <<"," << C_z_F[2] <<  endl;
    O_T_6=CalibrationPoseC1_P1; // This could obviously be optimized, but I let it like that since it's more easy to understand how calibration is computed
    F_T_6.setIdentity();
    ASetOfCoordinnates.setValue(0,0.0375,-0.0115); // This is Robotiq's LED position in the end effector's frame
    F_T_6.setOrigin(ASetOfCoordinnates);
    tf::Matrix3x3 TheOrientation;
    TheOrientation.setValue(C_x_F[0],C_y_F[0],C_z_F[0],C_x_F[1],C_y_F[1],C_z_F[1],C_x_F[2],C_y_F[2],C_z_F[2]);
    C1_T_F.setBasis(TheOrientation);
    ASetOfCoordinnates.setValue(P1x,P1y,P1z);
    C1_T_F.setOrigin(ASetOfCoordinnates);
    TmpPose1.mult(C1_T_F,F_T_6); // = C1_T_6
    TmpPose2.mult(TmpPose1,O_T_6.inverse()); // = C1_T_O
    O_T_C1=TmpPose2.inverse(); // O_T_C1 a.k.a the calibration matrix
    cout << endl << endl;
    cout << "All computed transformation matrices:" << endl;
    cout << endl << endl;
    cout << "O_T_6" << endl;
    cout << "The pose is: " << O_T_6.getBasis().getRow(0).x() << " " << O_T_6.getBasis().getRow(1).x() << " " << O_T_6.getBasis().getRow(2).x() << " " << O_T_6.getOrigin().getX() << endl;
    cout << "The pose is: " << O_T_6.getBasis().getRow(0).y() << " " << O_T_6.getBasis().getRow(1).y() << " " << O_T_6.getBasis().getRow(2).y() << " " << O_T_6.getOrigin().getY() << endl;
    cout << "The pose is: " << O_T_6.getBasis().getRow(0).z() << " " << O_T_6.getBasis().getRow(1).z() << " " << O_T_6.getBasis().getRow(2).z() << " " << O_T_6.getOrigin().getZ() << endl;
    cout << endl << endl;
    cout << "F_T_6" << endl;
    cout << "The pose is: " << F_T_6.getBasis().getRow(0).x() << " " << F_T_6.getBasis().getRow(1).x() << " " << F_T_6.getBasis().getRow(2).x() << " " << F_T_6.getOrigin().getX() << endl;
    cout << "The pose is: " << F_T_6.getBasis().getRow(0).y() << " " << F_T_6.getBasis().getRow(1).y() << " " << F_T_6.getBasis().getRow(2).y() << " " << F_T_6.getOrigin().getY() << endl;
    cout << "The pose is: " << F_T_6.getBasis().getRow(0).z() << " " << F_T_6.getBasis().getRow(1).z() << " " << F_T_6.getBasis().getRow(2).z() << " " << F_T_6.getOrigin().getZ() << endl;
    cout << endl << endl;
    cout << "C1_T_F" << endl;
    cout << "The pose is: " << C1_T_F.getBasis().getRow(0).x() << " " << C1_T_F.getBasis().getRow(1).x() << " " << C1_T_F.getBasis().getRow(2).x() << " " << C1_T_F.getOrigin().getX() << endl;
    cout << "The pose is: " << C1_T_F.getBasis().getRow(0).y() << " " << C1_T_F.getBasis().getRow(1).y() << " " << C1_T_F.getBasis().getRow(2).y() << " " << C1_T_F.getOrigin().getY() << endl;
    cout << "The pose is: " << C1_T_F.getBasis().getRow(0).z() << " " << C1_T_F.getBasis().getRow(1).z() << " " << C1_T_F.getBasis().getRow(2).z() << " " << C1_T_F.getOrigin().getZ() << endl;
    cout << endl << endl;
    cout << "C1_T_6" << endl;
    cout << "The pose is: " << TmpPose1.getBasis().getRow(0).x() << " " << TmpPose1.getBasis().getRow(1).x() << " " << TmpPose1.getBasis().getRow(2).x() << " " << TmpPose1.getOrigin().getX() << endl;
    cout << "The pose is: " << TmpPose1.getBasis().getRow(0).y() << " " << TmpPose1.getBasis().getRow(1).y() << " " << TmpPose1.getBasis().getRow(2).y() << " " << TmpPose1.getOrigin().getY() << endl;
    cout << "The pose is: " << TmpPose1.getBasis().getRow(0).z() << " " << TmpPose1.getBasis().getRow(1).z() << " " << TmpPose1.getBasis().getRow(2).z() << " " << TmpPose1.getOrigin().getZ() << endl;
    cout << endl << endl;
    cout << "C1_T_0" << endl;
    cout << "The pose is: " << TmpPose2.getBasis().getRow(0).x() << " " << TmpPose2.getBasis().getRow(1).x() << " " << TmpPose2.getBasis().getRow(2).x() << " " << TmpPose2.getOrigin().getX() << endl;
    cout << "The pose is: " << TmpPose2.getBasis().getRow(0).y() << " " << TmpPose2.getBasis().getRow(1).y() << " " << TmpPose2.getBasis().getRow(2).y() << " " << TmpPose2.getOrigin().getY() << endl;
    cout << "The pose is: " << TmpPose2.getBasis().getRow(0).z() << " " << TmpPose2.getBasis().getRow(1).z() << " " << TmpPose2.getBasis().getRow(2).z() << " " << TmpPose2.getOrigin().getZ() << endl;
    cout << endl << endl;
    cout << "O_T_C1" << endl;
    cout << "The pose is: " << O_T_C1.getBasis().getRow(0).x() << " " << O_T_C1.getBasis().getRow(1).x() << " " << O_T_C1.getBasis().getRow(2).x() << " " << O_T_C1.getOrigin().getX() << endl;
    cout << "The pose is: " << O_T_C1.getBasis().getRow(0).y() << " " << O_T_C1.getBasis().getRow(1).y() << " " << O_T_C1.getBasis().getRow(2).y() << " " << O_T_C1.getOrigin().getY() << endl;
    cout << "The pose is: " << O_T_C1.getBasis().getRow(0).z() << " " << O_T_C1.getBasis().getRow(1).z() << " " << O_T_C1.getBasis().getRow(2).z() << " " << O_T_C1.getOrigin().getZ() << endl;
    cout << endl << endl;

    ExportMatrixToCSV(O_T_C1,"/home/bdml/catkin_ws/CalibrationMatrices/O_T_C1.csv");
    cout << "The calibration matrix O_T_C1 has been successfully saved!" << endl;



/*      savinng twocamerqasextrisics 
    C1_T_C2.mult(O_T_C1.inverse(),O_T_C2);
    C2_T_C1.mult(O_T_C2.inverse(),O_T_C1);

    ExportMatrixToCSV(C1_T_C2,"/home/bdml/catkin_ws/CalibrationMatrices/C1_T_C2.csv");
    ExportMatrixToCSV(C2_T_C1,"/home/bdml/catkin_ws/CalibrationMatrices/C2_T_C1.csv");

    cout << "C1_T_C2: " << endl;
    cout << C1_T_C2.getBasis().getRow(0).x() << " " << C1_T_C2.getBasis().getRow(1).x() << " " << C1_T_C2.getBasis().getRow(2).x() << " " << C1_T_C2.getOrigin().getX() << endl;
    cout << C1_T_C2.getBasis().getRow(0).y() << " " << C1_T_C2.getBasis().getRow(1).y() << " " << C1_T_C2.getBasis().getRow(2).y() << " " << C1_T_C2.getOrigin().getY() << endl;
    cout << C1_T_C2.getBasis().getRow(0).z() << " " << C1_T_C2.getBasis().getRow(1).z() << " " << C1_T_C2.getBasis().getRow(2).z() << " " << C1_T_C2.getOrigin().getZ() << endl;
    cout << endl << endl;

    cout << "C2_T_C1: " << endl;
    cout << C2_T_C1.getBasis().getRow(0).x() << " " << C2_T_C1.getBasis().getRow(1).x() << " " << C2_T_C1.getBasis().getRow(2).x() << " " << C2_T_C1.getOrigin().getX() << endl;
    cout << C2_T_C1.getBasis().getRow(0).y() << " " << C2_T_C1.getBasis().getRow(1).y() << " " << C2_T_C1.getBasis().getRow(2).y() << " " << C2_T_C1.getOrigin().getY() << endl;
    cout << C2_T_C1.getBasis().getRow(0).z() << " " << C2_T_C1.getBasis().getRow(1).z() << " " << C2_T_C1.getBasis().getRow(2).z() << " " << C2_T_C1.getOrigin().getZ() << endl;
    cout << endl << endl;
*/
  }


  tfpose_init = robot.get_arm_position("base_link");
  orientEffectorTowardsTable(tfpose_init, tfpose_oriented); //Calculate matrix to orient gripper orthogonally with reference to the table
  ASetOfCoordinnates.setValue(-0.25,0.1,0.25); //Home Position
  CalibrationPoseC1_P1.setBasis(tfpose_oriented.getBasis());
  CalibrationPoseC1_P1.setOrigin(ASetOfCoordinnates);
  robot.move_to_point(CalibrationPoseC1_P1,"base_link",0.1,client); //move to the home position to prevent visual occlusion
  client.waitForResult();


  // Starting the vision/LED segmentation process:
//  int res = system("rosrun vision_based_picking FindRobotiqLedCam.py &");
//  cout << "The system call exit code was: " << res << endl;
//  sleep(INTER_COMMAND_DELAY); // give the vision process a bit of time to start
//  srv.request.cam_index=1;
//  srv.request.the_request="Start";



  while (ros::ok())
  {
    br1.sendTransform(tf::StampedTransform(O_T_C1, ros::Time::now(), "world","O_T_C1"));
    br1.sendTransform(tf::StampedTransform(O_T_C2, ros::Time::now(), "world","O_T_C2"));
    br1.sendTransform(tf::StampedTransform(C1_T_C2, ros::Time::now(), "world","C1_T_C2"));

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();
  std::cout << "Done!" << std::endl;
  return 0;
}


// A simple function computing a cross product
// jproberge on October 16th 2019
vector<double> cross_product(vector<double> V1, vector<double> V2)
{
  vector<double> V(3);
  V[0] = V1[1] * V2[2] - V1[2] * V2[1];
  V[1] = V1[2] * V2[0] - V1[0] * V2[2];
  V[2] = V1[0] * V2[1] - V1[1] * V2[0];

  return V;
}

void ExportMatrixToCSV(tf::Transform Matrix, string filename){
  std::ofstream myfile;
  myfile.open(filename.c_str());
  myfile << Matrix.getBasis().getRow(0).x() << "," << Matrix.getBasis().getRow(1).x() << "," << Matrix.getBasis().getRow(2).x() << "," << Matrix.getOrigin().getX() << "," << endl;
  myfile << Matrix.getBasis().getRow(0).y() << "," << Matrix.getBasis().getRow(1).y() << "," << Matrix.getBasis().getRow(2).y() << "," << Matrix.getOrigin().getY() << "," << endl;
  myfile << Matrix.getBasis().getRow(0).z() << "," << Matrix.getBasis().getRow(1).z() << "," << Matrix.getBasis().getRow(2).z() << "," << Matrix.getOrigin().getZ() << "," << endl;
  myfile.close();
}



/*****************************************************************************************
//Function: cmdOptionExists
//
//Description:  This function check a string starting at **begin up to **end and tries
//              to find the string defined by the argument &option. If it finds it, then
//              it returns true, otherwise it will return false.
//
//Author: Jean-Philippe Roberge
//Date: October 16th 2019
*****************************************************************************************/
bool cmdOptionExists(char** begin, char** end, const string& option)
{
    return find(begin, end, option) != end;
}

/****************************************************************************************
//Function: getCmdOption
//
//Description:  This function check a string starting at **begin up to **end and tries
//              to find the string defined by the argument &option. If it finds it, then
//              it returns a pointer pointing just after the string that was found.
//
//Author: Jean-Philippe Roberge
//Date: October 16th 2019
****************************************************************************************/
char* getCmdOption(char ** begin, char ** end, const string & option)
{
    char ** itr = find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}
