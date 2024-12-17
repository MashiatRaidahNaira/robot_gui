#include "robot_gui/robot_gui.h"
#include "geometry_msgs/Twist.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "std_srvs/Trigger.h"
#include <istream>

RobotGui::RobotGui() {
  ros::NodeHandle nh;
  twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  sub = nh.subscribe("/robot_info", 1000, &RobotGui::infoCallback, this);
  odom_sub = nh.subscribe("/odom", 1000, &RobotGui::odomCallback, this);
  get_distance_client = nh.serviceClient<std_srvs::Trigger>("/get_distance");
  distance_ = "0.00";
  reset_distance_client =
      nh.serviceClient<std_srvs::Trigger>("/reset_distance");
}

void RobotGui::infoCallback(
    const robotinfo_msgs::RobotInfo10FieldsConstPtr &msg) {
  info_msg = *msg;
}

void RobotGui::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odom_msg = *msg;
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

void RobotGui::robot_position_odometry(cv::Mat &frame) {
  // Position Parameters
  int theX = 20, theY = 500;
  double theTextFontScale = 0.5, thePrintFonstScale = 0.9;
  unsigned int theTextColor = 0xC0FFC0, thePrintColor = 0xFFFFFF;
  int theWX = theX, theWY = theY + 20;
  int theWidth = 110, theHeight = 110;

  cvui::text(frame, theX, theY,
             "Estimated robot position based off odometry:", theTextFontScale,
             theTextColor);

  cvui::window(frame, theWX, theWY, theWidth, theHeight, "X");
  cvui::printf(frame, theWX + 10, theWY + 50, thePrintFonstScale, thePrintColor,
               "%.2f", odom_msg.pose.pose.position.x);

  cvui::window(frame, theWX + 130, theWY, theWidth, theHeight, "Y");
  cvui::printf(frame, theWX + 140, theWY + 50, thePrintFonstScale,
               thePrintColor, "%.2f", odom_msg.pose.pose.position.y);

  cvui::window(frame, theWX + 260, theWY, theWidth, theHeight, "Z");
  cvui::printf(frame, theWX + 270, theWY + 50, thePrintFonstScale,
               thePrintColor, "%.2f", odom_msg.pose.pose.position.z);
}

void RobotGui::distance_travelled_service(cv::Mat &frame) {
  // Text, Call button, Window and printf Parameters
  int theX = 20, theY = 640;
  double theTextFontScale = 0.5, thePrintFonstScale = 0.9;
  unsigned int theTextColor = 0xC0FFC0, thePrintColor = 0xFFFFFF;
  int theBX = theX, theBY = theY + 20;
  int theBWidth = 110, theBHeight = 110;
  int theWX = theX + theBWidth + 20, theWY = theBY;
  int theWWidth = 230, theWHeight = theBHeight;

  cvui::text(frame, theX, theY, "Distance Travelled:", theTextFontScale,
             theTextColor);

  if (cvui::button(frame, theBX, theBY, theBWidth, theBHeight, "Call")) {
    if (get_distance_client.call(srv_req)) {
      distance_ = srv_req.response.message;
    } else {
      ROS_ERROR("Failed to call service /get_distance");
    }
  }

  cvui::window(frame, theWX, theWY, theWWidth, theWHeight,
               "Distance in meters:");
  cvui::printf(frame, theWX + 140, theWY + 60, thePrintFonstScale,
               thePrintColor, "%s", distance_.c_str());

  if (cvui::button(frame, theBX, theBY + 130, 360, 30, "Reset Distance")) {
    if (reset_distance_client.call(srv_req)) {
      distance_ = srv_req.response.message;
    } else {
      ROS_ERROR("Failed to call service /reset_distance");
    }
  }
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
    // Robot position (Odometry based)
    robot_position_odometry(frame);
    // Distance travelled service
    distance_travelled_service(frame);

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