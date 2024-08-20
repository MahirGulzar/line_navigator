#include "line_navigator/lane_follower.h"


LaneFollower::LaneFollower()
    : tf_listener_(tf_buffer_),
      sync_(rgb_image_sub_, depth_image_sub_, camera_info_rgb_sub_, camera_info_depth_sub_, 10)
{
    setupPublishersAndSubscribers();
    
    current_goal_index_ = 0;
    navigation_complete_ = false;
    paint_marking_ = false;

    // Prepare the point cloud message
    point_cloud_.header.frame_id = "map";
    sensor_msgs::PointField point_field_x;
    point_field_x.name = "x";
    point_field_x.offset = 0;
    point_field_x.datatype = sensor_msgs::PointField::FLOAT32;
    point_field_x.count = 1;
    sensor_msgs::PointField point_field_y;
    point_field_y.name = "y";
    point_field_y.offset = 4;
    point_field_y.datatype = sensor_msgs::PointField::FLOAT32;
    point_field_y.count = 1;
    sensor_msgs::PointField point_field_z;
    point_field_z.name = "z";
    point_field_z.offset = 8;
    point_field_z.datatype = sensor_msgs::PointField::FLOAT32;
    point_field_z.count = 1;
    point_cloud_.fields.clear();
    point_cloud_.fields.push_back(point_field_x);
    point_cloud_.fields.push_back(point_field_y);
    point_cloud_.fields.push_back(point_field_z);

    sync_.registerCallback(boost::bind(&LaneFollower::imageCallback, this, _1, _2, _3, _4));
    paint_marking_sub_ = nh_.subscribe("/paint_markings", 1, &LaneFollower::paintMarkingCallback, this);
}

LaneFollower::~LaneFollower()
{
    cv::destroyAllWindows();
}

void LaneFollower::setupPublishersAndSubscribers()
{
    image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/rgb/lane_detection", 10);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/lane_endpoints", 10);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacles_cloud", 10);

    move_base_result_sub_ = nh_.subscribe("/move_base/result", 10, &LaneFollower::navigationResultCallback, this);

    rgb_image_sub_.subscribe(nh_, "/camera/rgb/image_raw", 1);
    depth_image_sub_.subscribe(nh_, "/camera/depth/image_raw", 1);
    camera_info_rgb_sub_.subscribe(nh_, "/camera/rgb/camera_info", 1);
    camera_info_depth_sub_.subscribe(nh_, "/camera/depth/camera_info", 1);
}

void LaneFollower::imageCallback(const sensor_msgs::ImageConstPtr& rgb_data,
                                  const sensor_msgs::ImageConstPtr& depth_data,
                                  const sensor_msgs::CameraInfoConstPtr& camera_info_rgb,
                                  const sensor_msgs::CameraInfoConstPtr& camera_info_depth)
{
    cv_bridge::CvImagePtr cv_rgb_ptr, cv_depth_ptr;
    cv::Mat rgb_image, depth_image;

    try
    {
        cv_rgb_ptr = cv_bridge::toCvCopy(rgb_data, sensor_msgs::image_encodings::BGR8);
        cv_depth_ptr = cv_bridge::toCvCopy(depth_data, sensor_msgs::image_encodings::TYPE_16UC1);

        rgb_image = cv_rgb_ptr->image;
        depth_image = cv_depth_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("CvBridge Error: %s", e.what());
        return;
    }

    if (rgb_image.empty()) 
    {
        ROS_ERROR("Received an empty RGB image.");
        return;
    }

    camera_model_rgb_.fromCameraInfo(camera_info_rgb);
    camera_model_depth_.fromCameraInfo(camera_info_depth);

    cv::Mat gray_image;
    cv::cvtColor(rgb_image, gray_image, cv::COLOR_BGR2GRAY);

    cv::Mat binary_image;
    cv::threshold(gray_image, binary_image, 200, 255, cv::THRESH_BINARY);

    cv::Mat edges;
    cv::Canny(binary_image, edges, 50, 150);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 200, 10, 100);

    if (!lines.empty())
    {
        cv::Vec4i selected_line;
        int min_distance = std::numeric_limits<int>::max();
        int height = binary_image.rows;

        for (const auto &line : lines)
        {
            int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

            float slope = std::abs(static_cast<float>(y2 - y1) / (x2 - x1));
            if (slope <= 0.1) // Skip horizontal lines
                continue;

            int distance_to_bottom = std::min(height - y1, height - y2);

            if (distance_to_bottom < min_distance)
            {
                min_distance = distance_to_bottom;
                selected_line = line;
            }
        }

        if (selected_line != cv::Vec4i())
        {
            drawLineOnImage(rgb_image, selected_line);
            point_cloud_.header.stamp = ros::Time::now();
            point_cloud_pub_.publish(point_cloud_);

            // Fix the paint targets for navigation
            if (paint_marking_)
            {
                fixPaintTargets(selected_line, depth_image);
            }
        }
        
    }

    sensor_msgs::ImagePtr processed_image_msg;
    try
    {
        cv_bridge::CvImage cv_image;
        cv_image.image = rgb_image;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;

        processed_image_msg = cv_image.toImageMsg();

        processed_image_msg->header.stamp = ros::Time::now();
        processed_image_msg->header.frame_id = rgb_data->header.frame_id;

        image_pub_.publish(processed_image_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("CvBridge Error: %s", e.what());
    }
}

void LaneFollower::drawLineOnImage(cv::Mat &image, const cv::Vec4i &line)
{
    cv::line(image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 8);
    cv::circle(image, cv::Point(line[0], line[1]), 10, cv::Scalar(0, 0, 255), -1);
    cv::circle(image, cv::Point(line[2], line[3]), 10, cv::Scalar(0, 0, 255), -1);
}

void LaneFollower::fixPaintTargets(const cv::Vec4i &line, const cv::Mat &depth_image)
{
    if (goal_list_.empty())
    {
        // Get the transform from RGB camera frame to depth camera frame
        tf2::Transform transform_rgb_to_depth;
        try {
            geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform(camera_model_depth_.tfFrame(), camera_model_rgb_.tfFrame(), ros::Time(0));
            tf2::fromMsg(transformStamped.transform, transform_rgb_to_depth);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Convert RGB pixel to 3D ray in RGB camera frame
        cv::Point3f ray_start = camera_model_rgb_.projectPixelTo3dRay(cv::Point2d(line[0], line[1]));
        cv::Point3f ray_end = camera_model_rgb_.projectPixelTo3dRay(cv::Point2d(line[2], line[3]));

        // Transform rays from RGB camera frame to depth camera frame
        tf2::Vector3 ray_start_vec(ray_start.x, ray_start.y, ray_start.z);
        tf2::Vector3 ray_end_vec(ray_end.x, ray_end.y, ray_end.z);

        tf2::Vector3 ray_start_depth = transform_rgb_to_depth * ray_start_vec;
        tf2::Vector3 ray_end_depth = transform_rgb_to_depth * ray_end_vec;

        // Convert 3D coordinates in depth camera frame to 2D pixel coordinates
        cv::Point2d pixel_start = camera_model_depth_.project3dToPixel(cv::Point3f(ray_start_depth.x(), ray_start_depth.y(), ray_start_depth.z()));
        cv::Point2d pixel_end = camera_model_depth_.project3dToPixel(cv::Point3f(ray_end_depth.x(), ray_end_depth.y(), ray_end_depth.z()));

        // Clamp coordinates to be within image bounds
        pixel_start.x = std::max(0.0, std::min(pixel_start.x, (double)depth_image.cols - 1));
        pixel_start.y = std::max(0.0, std::min(pixel_start.y, (double)depth_image.rows - 1));
        pixel_end.x = std::max(0.0, std::min(pixel_end.x, (double)depth_image.cols - 1));
        pixel_end.y = std::max(0.0, std::min(pixel_end.y, (double)depth_image.rows - 1));

        // Retrieve depth values from depth image
        float z_start = depth_image.at<uint16_t>(cvRound(pixel_start.y), cvRound(pixel_start.x));
        float z_end = depth_image.at<uint16_t>(cvRound(pixel_end.y), cvRound(pixel_end.x));

        // Convert 2D pixel coordinates to 3D coordinates in RGB camera frame using depth values
        float x_start = ray_start.x * z_start;
        float y_start = ray_start.y * z_start;
        float x_end = ray_end.x * z_end;
        float y_end = ray_end.y * z_end;

        // Ensure that the start point is always below the end point, so that robot navigates from start to end
        if (z_start > z_end)
        {
            std::swap(x_start, x_end);
            std::swap(y_start, y_end);
            std::swap(z_start, z_end);
        }

        auto start_goal = getGoal(x_start, y_start, z_start, camera_model_rgb_.tfFrame());
        auto end_goal = getGoal(x_end, y_end, z_end, camera_model_depth_.tfFrame());

        // Calculate yaw angle from start to end goal
        float yaw_angle = std::atan2(end_goal.pose.position.y - start_goal.pose.position.y,
                                     end_goal.pose.position.x - start_goal.pose.position.x);

        tf::Quaternion orientation = tf::createQuaternionFromYaw(yaw_angle);

        start_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_angle);
        end_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_angle);

        goal_list_.push_back(start_goal);
        goal_list_.push_back(end_goal);
        marker_pub_.publish(getMarkers(goal_list_));
        goal_pub_.publish(start_goal);
    }
}

geometry_msgs::PoseStamped LaneFollower::getGoal(float x, float y, float z, const std::string &camera_frame)
{
    geometry_msgs::PoseStamped goal_msg;
    try
    {
        geometry_msgs::PoseStamped pose_camera;
        pose_camera.header.frame_id = camera_frame;
        pose_camera.header.stamp = ros::Time::now();
        pose_camera.pose.position.x = x;
        pose_camera.pose.position.y = y;
        pose_camera.pose.position.z = z;

        geometry_msgs::TransformStamped transform =
            tf_buffer_.lookupTransform("map", camera_frame, ros::Time(0), ros::Duration(1.0));

        tf2::doTransform(pose_camera, pose_camera, transform);

        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position = pose_camera.pose.position;
        goal_msg.pose.position.z = 0;
        goal_msg.pose.orientation.w = 1.0;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("Transform Error: %s", ex.what());
    }

    return goal_msg;
}

visualization_msgs::MarkerArray LaneFollower::getMarkers(const std::vector<geometry_msgs::PoseStamped> &goal_list)
{
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < goal_list.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = goal_list[i].pose;
        marker.scale.x = 0.4;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.id = i;

        marker_array.markers.push_back(marker);
    }

    return marker_array;
}

void LaneFollower::publishPointCloud()
{
    size_t interpolated_points = 100;
    size_t num_points = start_goal_list_.size() * interpolated_points;

    // Set point cloud dimensions
    point_cloud_.width = num_points;
    point_cloud_.height = 1;
    point_cloud_.header.frame_id = "map";
    
    point_cloud_.point_step = 12;
    point_cloud_.row_step = point_cloud_.point_step * point_cloud_.width;
    point_cloud_.data.resize(point_cloud_.width * point_cloud_.point_step);
    point_cloud_.is_dense = true;

    // Iterate through the start and end goal lists
    for (size_t i = 0; i < start_goal_list_.size(); ++i)
    {
        geometry_msgs::Point start = start_goal_list_[i].pose.position;
        geometry_msgs::Point end = end_goal_list_[i].pose.position;

        // Generate interpolated points
        for (size_t j = i * interpolated_points, k=0; j < ((i + 1) * interpolated_points); ++j, ++k)
        {
            
            float t = static_cast<float>(k) / ((interpolated_points + 20) - 1); // Normalized interpolation factor
            float x = start.x + t * (end.x - start.x);
            float y = start.y + t * (end.y - start.y);
            float z = start.z + t * (end.z - start.z);
            
            // Directly assign data
            float* data_ptr = reinterpret_cast<float*>(&point_cloud_.data[j * point_cloud_.point_step]);
            data_ptr[0] = x;
            data_ptr[1] = y;
            data_ptr[2] = z;
        }
    }
    // Publish the point cloud
    point_cloud_pub_.publish(point_cloud_);
}


void LaneFollower::paintMarkingCallback(const std_msgs::Empty::ConstPtr &msg)
{
    ROS_INFO("Intiating marking process...");
    paint_marking_ = true;
}

void LaneFollower::navigationResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &result_msg)
{
    if (paint_marking_)
    {
        int status = result_msg->status.status;
        if (status == 3) // 'Succeeded' status in actionlib
        {
            ++current_goal_index_;

            if (current_goal_index_ >= goal_list_.size())
            {
                navigation_complete_ = true;
                ROS_INFO("All lane marking points navigated!!");
                start_goal_list_.push_back(goal_list_.front());
                end_goal_list_.push_back(goal_list_.back());
                paint_marking_ = false;
                goal_list_.clear();
                current_goal_index_ = 0;

                // Publish obstacles point cloud
                publishPointCloud();
            }
            else
            {
                ROS_INFO("Reached a lane marking point..");
                goal_pub_.publish(goal_list_[current_goal_index_]);
            }
        }
    }
}
