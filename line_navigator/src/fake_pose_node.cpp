#include "line_navigator/fake_pose.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_pose_node");
    FakePose republisher;
    ros::spin();
    return 0;
}
