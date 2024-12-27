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
    robocam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "robotcam", 10, std::bind(&Viewer::robocam_callback, this, _1)
    );
    freecam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "freecam", 10, std::bind(&Viewer::freecam_callback, this, _1)
    );
    timer_ = this->create_wall_timer(
        50ms, std::bind(&Viewer::timer_callback, this));
    freecam_commands_ = this->create_publisher<ArrayMsg>("freecam_command", 10);

    init_SDL();
    last_update = std::chrono::system_clock::now();
}

void Viewer::robocam_callback(const sensor_msgs::msg::Image& msg)
{
    if (!robocam) return;

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

    for (int r=0; r<WINDOW_HEIGHT; r++) {
        for (int c=0; c<WINDOW_WIDTH; c++) {
            cv::Vec3b brg = cv_ptr->image.at<cv::Vec3b>(r, c);
            int color = 0xff000000 + (brg[2] << 16) + (brg[1] << 8) + brg[0];
            set_pixel(surface, c, r, color);
        }
    }

    renderFrame();
}

void Viewer::freecam_callback(const sensor_msgs::msg::Image& msg)
{
    if (robocam) return;

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

    for (int r=0; r<WINDOW_HEIGHT; r++) {
        for (int c=0; c<WINDOW_WIDTH; c++) {
            cv::Vec3b brg = cv_ptr->image.at<cv::Vec3b>(r, c);
            int color = 0xff000000 + (brg[2] << 16) + (brg[1] << 8) + brg[0];
            set_pixel(surface, c, r, color);
        }
    }

    renderFrame();
}

void Viewer::timer_callback()
{
    ArrayMsg msg;
    msg.data.push_back(0.0);
    msg.data.push_back(0.0);
    msg.data.push_back(0.0);
    msg.data.push_back(0.0);
    msg.data.push_back(0.0);

    int x, y;
    SDL_GetMouseState(&x, &y);

    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        switch (e.type) {

            case SDL_MOUSEBUTTONDOWN:
                buttons_pressed[0] = true;
                mouse_x = x;
                mouse_y = y;
                break;

            case SDL_MOUSEBUTTONUP:
                buttons_pressed[0] = false;
                break;

            case SDL_KEYDOWN:
            case SDL_KEYUP:
                switch (e.key.keysym.sym) {
                    case SDLK_w: buttons_pressed[1] = e.type == SDL_KEYDOWN; break;
                    case SDLK_a: buttons_pressed[2] = e.type == SDL_KEYDOWN; break;
                    case SDLK_s: buttons_pressed[3] = e.type == SDL_KEYDOWN; break;
                    case SDLK_d: buttons_pressed[4] = e.type == SDL_KEYDOWN; break;
                    case SDLK_f: buttons_pressed[5] = e.type == SDL_KEYDOWN; break;
                    case SDLK_SPACE: buttons_pressed[6] = e.type == SDL_KEYDOWN; break;
                    case SDLK_g: if (e.type == SDL_KEYDOWN) robocam = !robocam; break;
                }
                break;
        }
    }

    if (buttons_pressed[1]) msg.data[0] += 0.1;
    if (buttons_pressed[2]) msg.data[1] -= 0.1;
    if (buttons_pressed[3]) msg.data[0] -= 0.1;
    if (buttons_pressed[4]) msg.data[1] += 0.1;
    if (buttons_pressed[5]) msg.data[2] -= 0.1;
    if (buttons_pressed[6]) msg.data[2] += 0.1;

    if (buttons_pressed[0]) {
        msg.data[3] = (mouse_x - x) / 30.0;
        msg.data[4] = (mouse_y - y) / 60.0;

        mouse_x = x;
        mouse_y = y;
    }

    if (!robocam)
        freecam_commands_->publish(msg);
}

int Viewer::init_SDL()
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0){
        RCLCPP_INFO(this->get_logger(), "SDL_Init Error: %s", SDL_GetError());
        return 1;
    }
    window = SDL_CreateWindow("viewer_node", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
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
    SDL_RenderClear(renderer);
    SDL_UpdateTexture(texture, NULL, surface->pixels, surface->pitch);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);
}