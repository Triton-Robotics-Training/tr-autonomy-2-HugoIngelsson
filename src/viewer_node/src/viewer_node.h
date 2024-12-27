#ifndef VIEWER_NODE_SRC_VIEWER_NODE_H_
#define VIEWER_NODE_SRC_VIEWER_NODE_H_

#define WINDOW_HEIGHT 640
#define WINDOW_WIDTH 640

#define MAX_FPS 30

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"

#include "SDL.h"

using std::placeholders::_1;
using ArrayMsg = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Viewer : public rclcpp::Node {
  public:
    Viewer();
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr robocam_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr freecam_sub_;
    rclcpp::Publisher<ArrayMsg>::SharedPtr freecam_commands_;
    rclcpp::TimerBase::SharedPtr timer_;

    void robocam_callback(const sensor_msgs::msg::Image& msg);
    void freecam_callback(const sensor_msgs::msg::Image& msg);
    void timer_callback();

    SDL_Window* window = NULL;
    SDL_Renderer* renderer = NULL;
    SDL_Surface* surface = NULL;
    SDL_Texture* texture = NULL;
    int init_SDL();
    void set_pixel(SDL_Surface *surface, int x, int y, Uint32 pixel);
    void renderFrame();

    // 0 = mouse button
    // 1 = W, 2 = A, 3 = S, 4 = D
    // 5 = F, 6 = SPACE
    bool buttons_pressed[7] = {};
    int mouse_x = 0, mouse_y = 0;
    bool robocam = true;

    std::chrono::time_point<std::chrono::system_clock> last_update;
};

#endif // VIEWER_NODE_SRC_VIEWER_NODE_H