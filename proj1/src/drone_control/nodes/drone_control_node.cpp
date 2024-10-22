#include "drone_control/drone_control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControl>());
    rclcpp::shutdown();
    return 0;
}
