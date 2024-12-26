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
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"

#include "SDL.h"

using std::placeholders::_1;

class Viewer : public rclcpp::Node {
  public:
    Viewer();
  private:
    void image_callback(const sensor_msgs::msg::Image& msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    SDL_Window* window = NULL;
    SDL_Renderer* renderer = NULL;
    SDL_Surface* surface = NULL;
    SDL_Texture* texture = NULL;
    int init_SDL();
    void set_pixel(SDL_Surface *surface, int x, int y, Uint32 pixel);
    void renderFrame();

    std::chrono::time_point<std::chrono::system_clock> last_update;
};

#endif // VIEWER_NODE_SRC_VIEWER_NODE_H