#include "sol.h"

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Solution>());
  rclcpp::shutdown();
  return 0;
}

Solution::Solution() : Node("solution")
{
  RCLCPP_INFO(this->get_logger(), "Started sol.cpp");
  angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("desired_angle", 10);
  angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "current_angle", 10, std::bind(&Solution::angle_callback, this, _1)
  );
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "robotcam", 10, std::bind(&Solution::image_callback, this, _1)
  );
  point_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "scored_point", 10, std::bind(&Solution::point_callback, this, _1)
  );
}

void Solution::angle_callback(const std_msgs::msg::Float32& msg)
{
  cur_angle = msg.data;
  // RCLCPP_INFO(this->get_logger(), "Ping from angle; cur_angle = %f", cur_angle);
}

void Solution::image_callback(const sensor_msgs::msg::Image& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // is there a way to make this shared?
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_INFO(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  
  float avg_x = 0;
  int x_cnt = 0;
  for (int i=0; i*10<cv_ptr->image.cols; i++) {
    cv::Vec3b brg = cv_ptr->image.at<cv::Vec3b>(370, i*10);
    if (brg[2] >= 130 && brg[0] < 100 && brg[1] < 100) {
      avg_x += i*10;
      x_cnt++;
    }
  }

  std_msgs::msg::Float32 send;
  if (x_cnt == 0) {
    send.data = 0;
    RCLCPP_INFO(this->get_logger(), "No red found...");
    angle_pub_->publish(send);
  } else {
    send.data = -(avg_x / x_cnt - cv_ptr->image.cols / 2) 
        / cv_ptr->image.cols * 3.141592 / 2 + cur_angle;

    if (std::abs(avg_x / x_cnt - cv_ptr->image.cols / 2) < 20) {
      // do nothing, within range
    } 
    else {
      RCLCPP_INFO(this->get_logger(), "Go to angle: %f, with avg_x = %f", send.data, avg_x / x_cnt);
      angle_pub_->publish(send);
    }
  }

  angle_pub_->publish(send);
}

void Solution::point_callback(const std_msgs::msg::Empty& msg)
{
  RCLCPP_INFO(this->get_logger(), "Scored point!");
}