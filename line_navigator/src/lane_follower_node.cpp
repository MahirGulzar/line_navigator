#include "line_navigator/lane_follower.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_follower");
    LaneFollower lf;
    ros::spin();
    return 0;
}
