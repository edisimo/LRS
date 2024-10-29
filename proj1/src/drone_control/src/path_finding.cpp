#include "drone_control/path_finding.hpp"

PathFinding::PathFinding() : Node("path_finding_node")
{
    RCLCPP_INFO(this->get_logger(), "HELLO WORLD");
    mission_.ParsePoints("/home/lrs-ubuntu/LRS-FEI/mission_1_all.csv");
    mission_.DisplayPoints();
    map_.LoadMap("/home/lrs-ubuntu/LRS-FEI/maps/FEI_LRS_2D/dummy_map.txt");
}

MapLoading::MapLoading()
{
}

bool MapLoading::LoadMap(const std::string &map_name) {
    std::ifstream infile(map_name);
    if (!infile) {
        std::cerr << "Error: Cannot open map file.\n";
        return false;
    }

    std::string line;
    std::getline(infile, line); 
    infile >> width_ >> height_;

    map_data_.resize(height_, std::vector<int>(width_));

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            infile >> map_data_[y][x];
        }
    }
    PrintMap();
    std::cout << "Map loaded: " << width_ << "x" << height_ << std::endl;
    InflateMap(10);
    PrintMap();
    return true;
}

void MapLoading::PrintMap() {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            std::cout << map_data_[y][x]<< "";
        }
        std::cout << std::endl;
    }
}

void MapLoading::InflateCell(std::vector<std::vector<int>> &inflated_map, int x, int y, int inflationRadiusPx) {
    for (int dy = -inflationRadiusPx; dy <= inflationRadiusPx; ++dy) {
            for (int dx = -inflationRadiusPx; dx <= inflationRadiusPx; ++dx) {
                int newX = x + dx;
                int newY = y + dy;
                if (newX >= 0 && newX < width_ && newY >= 0 && newY < height_) {
                    inflated_map[newY][newX] = 0;
                }
            }
        }
}

void MapLoading::InflateMap(double inflation_radius_cm) {
    int inflation_radius_px = std::ceil(inflation_radius_cm / (resolution_ * 100));

    std::vector<std::vector<int>> inflated_map = map_data_;

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (map_data_[y][x] == 0) { 
                InflateCell(inflated_map, x, y, inflation_radius_px);
            }
        }
    }

    map_data_ = inflated_map;
    std::cout << "Map inflated by " << inflation_radius_cm << " cm." << std::endl;
 
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