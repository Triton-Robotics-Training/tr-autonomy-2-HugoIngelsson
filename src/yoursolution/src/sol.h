#ifndef YOUR_SOLUTION_SRC_SOL_H_
#define YOUR_SOLUTION_SRC_SOL_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"

using std::placeholders::_1;

class Solution : public rclcpp::Node {
 public:
  Solution();
 private:
  // your code here
  void angle_callback(const std_msgs::msg::Float32& msg);
  void image_callback(const sensor_msgs::msg::Image& msg);
  void point_callback(const std_msgs::msg::Empty& msg);
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr point_sub_;

  float cur_angle = 0.0;
};

#endif //YOUR_SOLUTION_SRC_SOL_H_
