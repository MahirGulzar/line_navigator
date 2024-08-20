#include "line_navigator/fake_pose.h"

FakePose::FakePose()
    : robot_name_("turtlebot3_waffle")
{
    ros::NodeHandle nh;
    pose_pub_ = nh.advertise<nav_msgs::Odometry>("/base_pose_ground_truth", 1);
    model_states_sub_ = nh.subscribe("/gazebo/model_states", 1, &FakePose::modelStatesCallback, this);
}

void FakePose::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // Search for the robot name in the list of model names
    auto it = std::find(msg->name.begin(), msg->name.end(), robot_name_);

    if (it != msg->name.end())
    {
        // Get the index of the robot
        size_t index = std::distance(msg->name.begin(), it);

        // Create the Odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose = msg->pose[index];
        odom.twist.twist = msg->twist[index];

        // Publish the Odometry message
        pose_pub_.publish(odom);
    }
}
