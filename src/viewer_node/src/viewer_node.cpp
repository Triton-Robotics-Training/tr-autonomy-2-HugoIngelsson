#include "viewer_node.h"

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Viewer>());
  rclcpp::shutdown();
  return 0;
}

Viewer::Viewer() : Node("viewer_node")
{
  RCLCPP_INFO(this->get_logger(), "Started viewer_node.cpp");
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "robotcam", 10, std::bind(&Viewer::image_callback, this, _1)
  );
  init_SDL();
  last_update = std::chrono::system_clock::now();
}

void Viewer::image_callback(const sensor_msgs::msg::Image& msg)
{
    SDL_Event PingStop;
    SDL_PollEvent(&PingStop);
    
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::chrono::duration<float> difference = now - last_update;
    if (difference.count() < 1.0 / MAX_FPS) {
        RCLCPP_INFO(this->get_logger(), "throttling video, small time difference since last \
            message: %f", difference.count());

        return;
    }
    last_update = now;
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "image dimensions: %d x %d", cv_ptr->image.cols, cv_ptr->image.rows);
    for (int r=0; r<WINDOW_HEIGHT; r++) {
        for (int c=0; c<WINDOW_WIDTH; c++) {
            cv::Vec3b brg = cv_ptr->image.at<cv::Vec3b>(r, c);
            int color = 0xff000000 + (brg[2] << 16) + (brg[1] << 8) + brg[0];
            set_pixel(surface, c, r, color);
        }
    }

    RCLCPP_INFO(this->get_logger(), "prepped pixels and ready to render");
    renderFrame();

    // float avg_x = 0;
    // int x_cnt = 0;
    // for (int i=0; i*10<cv_ptr->image.cols; i++) {
    //     cv::Vec3b brg = cv_ptr->image.at<cv::Vec3b>(370, i*10);
    //     if (brg[2] >= 130 && brg[0] < 100 && brg[1] < 100) {
    //         avg_x += i*10;
    //         x_cnt++;
    //     }
    // }

    RCLCPP_INFO(this->get_logger(), "read image");
}

int Viewer::init_SDL()
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0){
        RCLCPP_INFO(this->get_logger(), "SDL_Init Error: %s", SDL_GetError());
        return 1;
    }
    window = SDL_CreateWindow("robotcam", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == NULL){
        RCLCPP_INFO(this->get_logger(), "SDL_CreateWindow Error: %s", SDL_GetError());
        return 1;
    }
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == NULL) {
        RCLCPP_INFO(this->get_logger(), "Renderer could not be created! SDL Error: %s\n", SDL_GetError());
        return 1;
    } else {
        SDL_SetRenderDrawColor( renderer, 0x00, 0x00, 0x00, 0xFF );
    }
    surface = SDL_CreateRGBSurface(0, WINDOW_HEIGHT, WINDOW_WIDTH, 32, 0, 0, 0, 0);
    if (surface == NULL){
        RCLCPP_INFO(this->get_logger(), "SDL_CreateRGBSurface Error: %s", SDL_GetError());
        return 1;
    }
    texture = SDL_CreateTextureFromSurface(renderer, surface);
    if (texture == NULL){
        RCLCPP_INFO(this->get_logger(), "SDL_CreateTextureFromSurface Error: %s", SDL_GetError());
        return 1;
    }

    return 0;
}

void Viewer::set_pixel(SDL_Surface *surface, int x, int y, Uint32 pixel)
{
    Uint32 * const target_pixel = (Uint32 *) ((Uint8 *) surface->pixels
                                             + y * surface->pitch
                                             + x * surface->format->BytesPerPixel);
    *target_pixel = pixel;
}

void Viewer::renderFrame() {
    RCLCPP_INFO(this->get_logger(), "clearing render");
    SDL_RenderClear(renderer);
    RCLCPP_INFO(this->get_logger(), "updating texture");
    SDL_UpdateTexture(texture, NULL, surface->pixels, surface->pitch);
    RCLCPP_INFO(this->get_logger(), "rendering copy");
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    RCLCPP_INFO(this->get_logger(), "presenting new render");
    SDL_RenderPresent(renderer);
}