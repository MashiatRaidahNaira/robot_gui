#pragma once

#include "ros/service_client.h"
#include "ros/subscriber.h"
#define CVUI_DISABLE_COMPILATION_NOTICES
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_srvs/Trigger.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#define WINDOW_NAME "Robot Control GUI"

class RobotGui {
public:
  RobotGui();
  void info_render(cv::Mat &frame);
  void teleoperation_buttons(cv::Mat &frame);
  void current_velocity(cv::Mat &frame);
  void robot_position_odometry(cv::Mat &frame);
  void distance_travelled_service(cv::Mat &frame);
  void output();

private:
  ros::Subscriber sub;
  ros::Publisher twist_pub;
  geometry_msgs::Twist twist_msg;
  ros::Subscriber odom_sub;
  ros::ServiceClient get_distance_client;
  ros::ServiceClient reset_distance_client;
  std_srvs::Trigger srv_req;
  std::string distance_;
  nav_msgs::Odometry odom_msg;
  robotinfo_msgs::RobotInfo10Fields info_msg;
  void infoCallback(const robotinfo_msgs::RobotInfo10FieldsConstPtr &msg);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
};