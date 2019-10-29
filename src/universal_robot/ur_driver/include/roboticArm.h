#ifndef RoboticArm_H
#define RoboticArm_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <ur_msgs/GripperMove.h>
#include <ur_msgs/FollowCartesianTrajectoryAction.h>
//#include <tf_conversions/tf_eigen.h>

typedef actionlib::SimpleActionClient<ur_msgs::FollowCartesianTrajectoryAction> Client;

class RoboticArm
{
public:

    RoboticArm();
    void move_fingers(uint8_t pos, uint8_t  force, uint8_t speed, bool position_compenstation);
    void open_fingers(uint8_t force = 255, uint8_t speed = 255, bool position_compensation = false);
    void close_fingers(uint8_t force = 255, uint8_t speed = 255, bool position_compensation = false);
    void activate_gripper(void);
    void go_home(Client&);
    void go_bin(Client&);
    void go_bin_approach(Client&, tf::Transform);
    void move_to_point(tf::Transform tf_point, std::string frameID, double speed, Client & client);
    bool gripper_has_object(void);
    void execute_trajectory(const std::vector<tf::Transform> &trajectory, double velocity, std::string frameID);


    tf::StampedTransform get_arm_position_in_camera_frame();
    tf::StampedTransform get_teached_arm_position();

    tf::StampedTransform get_arm_position(std::string refFrameID);


    tf::Transform tf_home;
    tf::Transform tf_bin;
    tf::StampedTransform get_arm_position_base(std::string refFrameID);


private:

    bool has_object;
    tf::Transform tf_bin_approach;
};

#endif
