#ifndef LANE_FOLLOWER_H
#define LANE_FOLLOWER_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>

class LaneFollower
{
public:
    LaneFollower();
    ~LaneFollower();

    void imageCallback(const sensor_msgs::ImageConstPtr& rgb_data,
                        const sensor_msgs::ImageConstPtr& depth_data,
                        const sensor_msgs::CameraInfoConstPtr& camera_info_rgb,
                        const sensor_msgs::CameraInfoConstPtr& camera_info_depth);

    void navigationResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& result_msg);

private:
    // Class Attributes
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel camera_model_rgb_;
    image_geometry::PinholeCameraModel camera_model_depth_;
    std::vector<geometry_msgs::PoseStamped> goal_list_;
    std::vector<geometry_msgs::PoseStamped> start_goal_list_;
    std::vector<geometry_msgs::PoseStamped> end_goal_list_;
    std::vector<sensor_msgs::PointField> point_fields_;
    int current_goal_index_;
    bool navigation_complete_, paint_marking_;
    sensor_msgs::PointCloud2 point_cloud_;
    
    // Publishers
    ros::Publisher image_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher point_cloud_pub_;

    // Subscribers
    ros::Subscriber camera_info_sub, move_base_result_sub_, paint_marking_sub_;
    message_filters::Subscriber<sensor_msgs::Image> rgb_image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_depth_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync_;

    // Class Methods
    void publishPointCloud();

    void paintMarkingCallback(const std_msgs::Empty::ConstPtr& msg);

    geometry_msgs::PoseStamped getGoal(float x, float y, float z, const std::string& camera_frame);

    visualization_msgs::MarkerArray getMarkers(const std::vector<geometry_msgs::PoseStamped>& goal_list);

    void drawLineOnImage(cv::Mat& image, const cv::Vec4i& line);

    void fixPaintTargets(const cv::Vec4i& line, const cv::Mat& depth_image);

    void setupPublishersAndSubscribers();
};

#endif // LANE_FOLLOWER_H
