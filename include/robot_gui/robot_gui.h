#pragma once

#define CVUI_DISABLE_COMPILATION_NOTICES
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#include "geometry_msgs/Twist.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#define WINDOW_NAME "Robot Control GUI"

class RobotGui {
public:
  RobotGui();
  void info_render(cv::Mat &frame);
  void teleoperation_buttons(cv::Mat &frame);
  void output();

private:
  ros::Subscriber sub;
  ros::Publisher twist_pub;
  geometry_msgs::Twist twist_msg;
  robotinfo_msgs::RobotInfo10Fields info_msg;
  void infoCallback(const robotinfo_msgs::RobotInfo10FieldsConstPtr &msg);
};