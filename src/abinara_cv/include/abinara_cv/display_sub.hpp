#ifndef DISPLAY_SUB_HPP
#define DISPLAY_SUB_HPP

#include <ros/ros.h>
#include <abinara_cv/VisionInfo.h>
#include <iostream>

void visionCallback(const abinara_cv::VisionInfo::ConstPtr &msg);

#endif
