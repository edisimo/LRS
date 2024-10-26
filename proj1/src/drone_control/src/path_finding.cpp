#include "drone_control/path_finding.hpp"

PathFinding::PathFinding() : Node("path_finding_node")
{
    RCLCPP_INFO(this->get_logger(), "HELLO WORLD");
    mission_.ParsePoints("/home/lrs-ubuntu/LRS-FEI/mission_1_all.csv");
    mission_.DisplayPoints();
}

MapLoading::MapLoading()
{
}

MissionParser::MissionParser()
{
}

void MissionParser::ParsePoints(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;   
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.size() == 5) {
            double x = std::stod(tokens[0]);
            double y = std::stod(tokens[1]);
            double z = std::stod(tokens[2]);
            std::string precision = tokens[3];
            std::string command = tokens[4];
            points_.emplace_back(x, y, z, precision, command);
        } 
        else {
            // TODO: find out how to use a dummy logger
            // rclcpp::Logger logger("");
            // RCLCPP_ERROR(logger, "Failed to parse file");
            std::cout << "Parsing failed" << std::endl;
            return;
        }
    }
    std::cout << "Finished" << std::endl;
}

void MissionParser::DisplayPoints() {
    for (const auto& point : points_) {
        std::cout << "Point: (" << point.x_<< ", " << point.y_ << ", " << point.z_
                  << "), Precision: " << point.precision_
                  << ", Command: " << (point.command_.empty() ? "None" : point.command_)
                  << std::endl;
    }
}