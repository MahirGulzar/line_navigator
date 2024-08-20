#ifndef FAKE_POSE_H
#define FAKE_POSE_H

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <string>

class FakePose
{
public:
    FakePose();
    
private:
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

    ros::Publisher pose_pub_;
    ros::Subscriber model_states_sub_;
    std::string robot_name_;
};

#endif // FAKE_POSE_H
