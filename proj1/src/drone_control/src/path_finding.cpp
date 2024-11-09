#include "drone_control/path_finding.hpp"

PathFinding::PathFinding() : Node("path_finding_node")
{
    RCLCPP_INFO(this->get_logger(), "HELLO WORLD");
    mission_.ParsePoints("/home/lrs-ubuntu/LRS-FEI/mission_1_all.csv");
    mission_.DisplayPoints();
    // map_.LoadMap("/home/lrs-ubuntu/LRS-FEI/maps/FEI_LRS_2D/dummy_map.txt", 0);
    // map_.LoadMap("/home/lrs-ubuntu/LRS-FEI/maps/FEI_LRS_2D/dummy_map2.txt", 1);
    map_.LoadAllMaps("/home/lrs-ubuntu/LRS-FEI/maps/FEI_LRS_2D/");
    double query_x = 0;
    double query_y = 0;
    int z_level = 25;
    int cell_value = map_.GetCellValue(query_x, query_y, z_level);
    std::cout << "Cell value at (" << query_x << ", " << query_y << ") on z-level " << z_level << ": " << cell_value << std::endl;
    query_x = 9*0.05;
    query_y = 9*0.05;
    cell_value = map_.GetCellValue(query_x, query_y, z_level);
    std::cout << "Cell value at (" << query_x << ", " << query_y << ") on z-level " << z_level << ": " << cell_value << std::endl;
}

MapLoading::MapLoading()
{
}

void MapLoading::LoadAllMaps(const std::string &directory_path) {
    namespace fs = std::filesystem;

    try {
        for (const auto& entry : fs::directory_iterator(directory_path)) {
            if (entry.is_regular_file() && entry.path().extension() == ".pgm") {
                std::string filename = entry.path().filename().string();
                
                if (filename.rfind("map_", 0) == 0) {
                    try {
                        int height_cm = std::stoi(filename.substr(4, 3));
                        int z_level = height_cm;

                        if (LoadMap(entry.path().string(), z_level)) {
                            std::cout << "Loaded map at z-level " << z_level << " from " << filename << std::endl;
                        } else {
                            std::cerr << "Failed to load map from " << filename << std::endl;
                        }
                    } catch (const std::exception&) {
                        std::cerr << "Warning: Invalid filename format or load error: " << filename << std::endl;
                    }
                }
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    }
}

bool MapLoading::LoadMap(const std::string &map_name, int z_level) {
    std::ifstream infile(map_name);
    if (!infile) {
        std::cerr << "Error: Cannot open map file.\n";
        return false;
    }

    std::string line;
    std::getline(infile, line); 
    infile >> width_ >> height_;

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int value;
            infile >> value;
            map_data_[Key(x, height_-1-y, z_level)] = value;
        }
    }
    // PrintMap(z_level);
    std::cout << "Map loaded: " << width_ << "x" << height_ << " at z=" << z_level << std::endl;
    // InflateMap(5, z_level);
    // PrintMap(z_level);
    return true;
}

void MapLoading::PrintMap(int z_level) {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            Key key = Key(x, y, z_level);
            int value = map_data_[key];
            std::cout << value << "";        
        }
        std::cout << std::endl;
    }
}

void MapLoading::InflateCell(Map3D &inflated_map, int x, int y, int z, int inflationRadiusPx) {
    for (int dy = -inflationRadiusPx; dy <= inflationRadiusPx; ++dy) {
            for (int dx = -inflationRadiusPx; dx <= inflationRadiusPx; ++dx) {
                int newX = x + dx;
                int newY = y + dy;
                if (newX >= 0 && newX < width_ && newY >= 0 && newY < height_) {
                    Key key = Key(newX, newY, z);
                    inflated_map[key] = 0;
                }
            }
        }
}

void MapLoading::InflateMap(double inflation_radius_cm, int z_level) {
    int inflation_radius_px = std::ceil(inflation_radius_cm / (resolution_ * 100));

    Map3D inflated_map = map_data_;

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            Key key = Key(x, y, z_level);
            if (map_data_[key] == 0) { 
                InflateCell(inflated_map, x, y, z_level, inflation_radius_px);            }
        }
    }

    map_data_ = inflated_map;
    std::cout << "Map inflated by " << inflation_radius_cm << " cm at z=" << z_level << "." << std::endl; 
}

std::pair<int, int> MapLoading::ConvertToMapIndices(double x, double y) {
    int x_index = static_cast<int>(std::round(x / resolution_));
    int y_index = static_cast<int>(std::round(y / resolution_));
    return {x_index, y_index};
}

int MapLoading::GetCellValue(double x, double y, int z_level) {
    auto [x_index, y_index] = ConvertToMapIndices(x, y);
    Key key = Key(x_index, y_index, z_level);
    
    if (map_data_.find(key) != map_data_.end()) {
        return map_data_[key];
    } else {
        std::cerr << "Error: Coordinate (" << x << ", " << y << ") is out of map bounds." << std::endl;
        return -1; 
    }
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