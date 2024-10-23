#include "drone_control/path_finding.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFinding>());
    rclcpp::shutdown();
    return 0;
}
