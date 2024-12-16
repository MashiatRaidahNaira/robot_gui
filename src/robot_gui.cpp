#include "robot_gui/robot_gui.h"
#include "ros/init.h"
#include "ros/node_handle.h"

RobotGui::RobotGui() {
  ros::NodeHandle nh;
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

void RobotGui::output() {

  cvui::init(WINDOW_NAME);

  // Create a frame
  cv::Mat frame = cv::Mat(900, 440, CV_8UC3);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);
    // General Info Area
    info_render(frame);
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