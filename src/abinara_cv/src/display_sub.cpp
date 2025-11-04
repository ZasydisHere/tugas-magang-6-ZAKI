#include "abinara_cv/display_sub.hpp"

void visionCallback(const abinara_cv::VisionInfo::ConstPtr &msg)
{
    ROS_INFO("FPS: %.2f | Shape: %s | Color: %s",
             msg->fps,
             msg->shape.c_str(),
             msg->color.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/vision/info", 10, visionCallback);

    ROS_INFO("Vision Subscriber Running...");

    ros::spin();
    return 0;
}
