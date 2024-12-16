#include "robot_gui/robot_gui.h"
#include "geometry_msgs/Twist.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include <istream>

RobotGui::RobotGui() {
  ros::NodeHandle nh;
  twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  sub = nh.subscribe("/robot_info", 1000, &RobotGui::infoCallback, this);
}

void RobotGui::infoCallback(
    const robotinfo_msgs::RobotInfo10FieldsConstPtr &msg) {
  info_msg = *msg;
}

void RobotGui::info_render(cv::Mat &frame) {
  // window parameters
  int theWX = 20, theWY = 20, theWidth = 400, theHeight = 200;
  const cv::String theTitle = "General Info";
  // printf parameters
  int thePX = 40, thePY = 40;
  double theFontScale = 0.4;
  unsigned int theColor = 0xC0FFEE;
  int lineSpacing = 18;

  std::vector<std::string> general_info = {
      info_msg.data_field_01, info_msg.data_field_02, info_msg.data_field_03,
      info_msg.data_field_04, info_msg.data_field_05, info_msg.data_field_06,
      info_msg.data_field_07, info_msg.data_field_08, info_msg.data_field_09,
      info_msg.data_field_10};

  cvui::window(frame, theWX, theWY, theWidth, theHeight, theTitle);

  for (const auto &i : general_info) {
    cvui::printf(frame, thePX, thePY, theFontScale, theColor, i.c_str());
    // Move down for the next line
    thePY += lineSpacing;
  }
}

void RobotGui::teleoperation_buttons(cv::Mat &frame) {
  // Button parameters
  int width = 80, height = 50;
  int fx = 170, fy = 240;
  int sx = 170, sy = 300;
  int bx = 170, by = 360;
  int rx = 260, ry = 300;
  int lx = 80, ly = 300;

  if (cvui::button(frame, fx, fy, width, height, "Forward")) {
    twist_msg.linear.x += 0.5;
  }

  if (cvui::button(frame, sx, sy, width, height, "Stop")) {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
  }

  if (cvui::button(frame, bx, by, width, height, "Backward")) {
    twist_msg.linear.x -= 0.5;
  }

  if (cvui::button(frame, rx, ry, width, height, "Right")) {
    twist_msg.angular.z -= 0.5;
  }

  if (cvui::button(frame, lx, ly, width, height, "Left")) {
    twist_msg.angular.z += 0.5;
  }

  twist_pub.publish(twist_msg);
}

void RobotGui::current_velocity(cv::Mat &frame) {
  // Parameters
  int lin_x = 20, lin_y = 420;
  int ang_x = 220, ang_y = 420;
  int width = 185, height = 60;

  cvui::window(frame, lin_x, lin_y, width, height, "Linear velocity:");
  cvui::printf(frame, lin_x + 60, lin_y + 30, 0.5, 0xff0000, "%.2f m/sec",
               twist_msg.linear.x);

  cvui::window(frame, ang_x, ang_y, width, height, "Angular velocity:");
  cvui::printf(frame, ang_x + 60, ang_y + 30, 0.5, 0xff0000, "%.2f rad/sec",
               twist_msg.angular.z);
}

void RobotGui::output() {

  cvui::init(WINDOW_NAME);

  // Create a frame
  cv::Mat frame = cv::Mat(900, 440, CV_8UC3);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // General Info Area
    info_render(frame);
    // Teleoperation Buttons
    teleoperation_buttons(frame);
    // Current velocities
    current_velocity(frame);

    // Show final result
    cvui::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Update Callbacks
    ros::spinOnce();
  }
}