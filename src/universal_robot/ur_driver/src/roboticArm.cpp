#include "roboticArm.h"

#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

#include <ur_msgs/GripperMove.h>
#include <ur_msgs/FollowCartesianTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>


typedef ur_msgs::FollowCartesianTrajectoryGoal CartesianTrajectory;

/**
  This function is used to open the fingers of the gripper. It should exit the function only when the gripper has finished opening.
*/

RoboticArm::RoboticArm()
{
    tf::Vector3 origin;
    tf::Matrix3x3 basis;

    origin.setValue(0.334122,0.32483,0.605199);
    basis.setRPY(-3.14,0,0);

    this->tf_home.setOrigin(origin);
    this->tf_home.setBasis(basis);

    origin.setValue(0.400, -0.225, 0.420);
    basis.setRPY(-3.0830, 0.0138, 1.9170);

    this->tf_bin_approach.setOrigin(origin);
    this->tf_bin_approach.setBasis(basis);

    origin.setZ(0.350);
    this->tf_bin.setOrigin(origin);
    this->tf_bin.setBasis(basis);

}

void RoboticArm::move_fingers(uint8_t pos, uint8_t force, uint8_t speed, bool position_compenstation){
    ros::NodeHandle n;
    ur_msgs::GripperMove srv;
    ros::ServiceClient gripper_client = n.serviceClient<ur_msgs::GripperMove>("gripper/move");

    srv.request.force = force;
    srv.request.speed = speed;
    srv.request.pos = pos;
    srv.request.position_compensation = position_compenstation;

    ROS_INFO("Wait for service gripper/move");
    gripper_client.waitForExistence();
    ROS_INFO("Calling service gripper/move");

    if (!gripper_client.call(srv))
    {
        this->has_object = false;
        ROS_ERROR("Failed to call service gripper/move");
    }
    else
    {
        this->has_object = srv.response.has_object;
    }
}

void RoboticArm::open_fingers(uint8_t force, uint8_t speed, bool position_compensation){
    this->move_fingers(0, force, speed, position_compensation);
}


/**
  This function is used to close the fingers of the gripper (for grasping an object). It should exit the function only when the gripper is closed.
*/
void RoboticArm::close_fingers(uint8_t force, uint8_t speed, bool position_compensation){
    this->move_fingers(255, force, speed, position_compensation);
}

void RoboticArm::activate_gripper(void)
{
    ros::NodeHandle n;
    std_srvs::Empty e;
    ros::ServiceClient gripper_client = n.serviceClient<std_srvs::Empty>("gripper/activate");

    ROS_INFO("Wait for service gripper/activate");
    gripper_client.waitForExistence();
    ROS_INFO("Calling service gripper/activate");

    if (!gripper_client.call(e))
    {
        ROS_ERROR("Failed to call service gripper/activate");
    }
}

/**
  This function sends the robot back to a "home" position where the robot can reach any object on the table
*/
void RoboticArm::go_home(Client & client){
    this->move_to_point(this->tf_home,"base_link",0.6, client);
}

/**
  This function sends the robot back to a "bin" where a grasped object can be released.
*/
void RoboticArm::go_bin(Client & client){
    this->move_to_point(this->tf_bin,"base_link",0.1, client);
}

/**
  This function sends the robot back to a "bin" where a grasped object can be released.
*/
void RoboticArm::go_bin_approach(Client & client, tf::Transform tf_pick_pose){
    this->move_to_point(tf_pick_pose,"base_link",0.6, client);
}


/**
  This function moves the end-effector to the desired pose.
  The movement will be linear (moveL)
  @param tf_point The pose where the arm has to be moved.
         frameID  Reference frame of the movement. Typically "base" or "base_link"
*/
void RoboticArm::move_to_point(tf::Transform tf_point, std::string frameID, double speed, Client & client){

    CartesianTrajectory trajectory;
    ROS_INFO("WAITING FOR SERVER");
    client.waitForServer();
    ROS_INFO("SERVER AVAILABLE");

    if (client.isServerConnected()) {

            trajectory.header.frame_id = frameID;
            trajectory.velocity = speed;
            trajectory.poses = std::vector<geometry_msgs::Pose>(1);
            trajectory.poses[0].position.x = tf_point.getOrigin().getX();
            trajectory.poses[0].position.y = tf_point.getOrigin().getY();
            trajectory.poses[0].position.z = tf_point.getOrigin().getZ();
            trajectory.poses[0].orientation.x = tf_point.getRotation().getY();
            trajectory.poses[0].orientation.y = tf_point.getRotation().getX();
            trajectory.poses[0].orientation.z = tf_point.getRotation().getW();
            trajectory.poses[0].orientation.w = -tf_point.getRotation().getZ();

            ROS_INFO("%f %f %f %f %f %f %f", trajectory.poses[0].position.x,
                                                                      trajectory.poses[0].position.y,
                                                                      trajectory.poses[0].position.z,
                                                                      trajectory.poses[0].orientation.x,
                                                                      trajectory.poses[0].orientation.y,
                                                                      trajectory.poses[0].orientation.z,
                                                                      trajectory.poses[0].orientation.w);


    }

    client.sendGoal(trajectory);
    if (client.waitForResult(ros::Duration(120.0)))
    {
      ROS_INFO("movel success");
    }
    else
    {
      ROS_INFO("movel failure");

    }
}

bool RoboticArm::gripper_has_object(void)
{
    return this->has_object;
}


/**
  This function moves the end-effector to the desired positions.
  The movements will be linear (moveL)
  @param trajectory Array of poses where the arm has to be moved.
         velocity   Speed of the movement
         frameID    Reference frame of the movement. Typically "base" or "base_link"
*/
void RoboticArm::execute_trajectory(const std::vector<tf::Transform> &trajectory, double velocity
                                    , std::string frameID)
{
    CartesianTrajectory cart_trajectory;
    Client client ("follow_cart_trajectory");
    ROS_INFO("WAITING FOR SERVER");
    client.waitForServer();
    ROS_INFO("SERVER AVAILABLE");

    if (trajectory.empty())
    {
        ROS_WARN("Empty trajectory");
        return;
    }

    cart_trajectory.header.frame_id = frameID;
    cart_trajectory.velocity = velocity;
    cart_trajectory.poses = std::vector<geometry_msgs::Pose>(trajectory.size());

    ROS_INFO("TRANSLATING TRAJECTORY");
    for (int i = 0; i < cart_trajectory.poses.size(); i++)
    {
        cart_trajectory.poses[i].position.x = trajectory[i].getOrigin().getX();
        cart_trajectory.poses[i].position.y = trajectory[i].getOrigin().getY();
        cart_trajectory.poses[i].position.z = trajectory[i].getOrigin().getZ();
        cart_trajectory.poses[0].orientation.x = trajectory[i].getRotation().getY();
        cart_trajectory.poses[0].orientation.y = trajectory[i].getRotation().getX();
        cart_trajectory.poses[0].orientation.z = trajectory[i].getRotation().getW();
        cart_trajectory.poses[0].orientation.w = -trajectory[i].getRotation().getZ();
    }

    ROS_INFO("SENDING GOAL");
    client.sendGoal(cart_trajectory);

    if (client.waitForResult(ros::Duration(30.0)))
    {
      ROS_INFO("movel success");
    }
    else
    {
      ROS_INFO("movel failure");
    }
}

/**
  This function gets the end-effector position in camera_rgb_frame.
*/
tf::StampedTransform RoboticArm::get_arm_position_in_camera_frame(){

    // Get the transform "camera_rgb_frame" -> "TF_UR_END_EFFECTOR". You can rename "TF_UR_END_EFFECTOR" in the following code for the real tf.
    // For this transformation to exist, an extrinsic calibration between the arm and camera must have been done. Once the tf is known, it can be published as follows in a launch file :
    // <node pkg="tf" type="static_transform_publisher" name="UR_KINECT" args=" x y z rx ry rz UR_ROBOT_BASE SOME_CAMERA_FRAME 100" />

    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform("camera_rgb_frame","tool0",ros::Time(0),ros::Duration(1.0));
    listener.lookupTransform("camera_rgb_frame","tool0",ros::Time(0),transform);
    return transform;
}


/**
  This function gets the end-effector position in camera_rgb_frame in teaching mode.
*/
tf::StampedTransform RoboticArm::get_teached_arm_position(){
    return get_arm_position_in_camera_frame();
}



/**
  This function gets the end-effector position in the specified frame.
  @param refFrameID Frame identification of the reference frame
*/

tf::StampedTransform RoboticArm::get_arm_position(std::string refFrameID)
{


    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform(refFrameID,"tool0",ros::Time(0),ros::Duration(1.0));
    listener.lookupTransform(refFrameID, "tool0",ros::Time(0),transform);
    return transform;
}


tf::StampedTransform RoboticArm::get_arm_position_base(std::string refFrameID)
{


    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform(refFrameID,"base_link",ros::Time(0),ros::Duration(1.0));  //was tool0 before
    listener.lookupTransform(refFrameID, "base_link",ros::Time(0),transform);
    return transform;
}



