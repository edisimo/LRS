#include "drone_control/path_finding.hpp"

PathFinding::PathFinding() : Node("path_finding_node")
{
    RCLCPP_INFO(this->get_logger(), "HELLO WORLD");
    mission_.ParsePoints("/home/lrs-ubuntu/LRS-FEI/mission_1_all.csv");
    mission_.DisplayPoints();
    // map_.LoadMap("/home/lrs-ubuntu/LRS-FEI/maps/FEI_LRS_2D/dummy_map.txt", 0);
    // map_.LoadMap("/home/lrs-ubuntu/LRS-FEI/maps/FEI_LRS_2D/dummy_map2.txt", 1);
    map_.LoadAllMaps("/home/lrs-ubuntu/LRS-FEI/maps/FEI_LRS_2D/");

    // Example usage: Get cell value at specific coordinates and z-level
    double x = 1.0; // x coordinate in meters
    double y = 2.0; // y coordinate in meters
    double z_level = 25.5; // z-level in meters or your unit

    int cell_value = map_.GetCellValue(x, y, z_level);
    if (cell_value != -1)
    {
        RCLCPP_INFO(this->get_logger(), "Cell value at (%f, %f, %f): %d", x, y, z_level, cell_value);
    }

    // Get list of available z-levels
    std::vector<double> z_levels = map_.GetAvailableZLevels();
    RCLCPP_INFO(this->get_logger(), "Available z-levels:");
    for (double z : z_levels)
    {
        RCLCPP_INFO(this->get_logger(), "%f", z);
    }
}

MapLoading::MapLoading()
{
}

void MapLoading::LoadAllMaps(const std::string &directory_path) {
    namespace fs = std::filesystem;
    for (const auto& entry : fs::directory_iterator(directory_path))
    {
        if (entry.path().extension() == ".pgm")
        {
            std::string filename = entry.path().filename().string();
            size_t pos1 = filename.find("map_");
            size_t pos2 = filename.find(".pgm");
            std::string z_str = filename.substr(pos1 + 4, pos2 - (pos1 + 4));
            double z_level_cm = std::stod(z_str);
            double z_level_m = z_level_cm / 100.0;
            nav_msgs::msg::OccupancyGrid map;
            LoadMapPGM(entry.path().string(), map, z_level_m);
            maps_[z_level_m] = map;
        }
    }
}

bool MapLoading::LoadMapPGM(const std::string &map_name, nav_msgs::msg::OccupancyGrid &map, double z_level) {
    std::ifstream infile(map_name, std::ios::binary);
    std::string line;
    int width, height, max_value;
    double resolution = resolution_;
    std::vector<double> origin = {0.0, 0.0, z_level};

    std::getline(infile, line);
    bool is_binary = (line == "P5");
    while (std::getline(infile, line)) {
        if (line[0] != '#') break;
    }
    std::istringstream iss(line);
    iss >> width >> height;
    infile >> max_value;
    infile.get();

    map.info.resolution = resolution;
    map.info.width = width;
    map.info.height = height;
    map.info.origin.position.x = origin[0];
    map.info.origin.position.y = origin[1];
    map.info.origin.position.z = origin[2];
    map.info.origin.orientation.w = 1.0;
    map.data.resize(width * height);

    if (is_binary)
    {
        std::vector<uint8_t> pgm_data(width * height);
        infile.read(reinterpret_cast<char*>(pgm_data.data()), width * height);
        for (int y = 0; y < height; ++y)
        {
            int inverted_y = height - 1 - y;
            for (int x = 0; x < width; ++x)
            {
                size_t pgm_index = y * width + x;
                size_t map_index = inverted_y * width + x;
                uint8_t value = pgm_data[pgm_index];
                map.data[map_index] = (value == 0) ? 100 : (value == max_value) ? 0 : -1;
            }
        }
    }
    else
    {
        for (int y = 0; y < height; ++y)
        {
            int inverted_y = height - 1 - y;
            for (int x = 0; x < width; ++x)
            {
                int value;
                infile >> value;
                size_t map_index = inverted_y * width + x;
                map.data[map_index] = (value == 0) ? 100 : (value == max_value) ? 0 : -1;
            }
        }
    }
    //TODO: constant inflation radius
    double inflation_radius_cm = 15.0;
    InflateObstacles(map, inflation_radius_cm);
    return true;
}

void MapLoading::InflateObstacles(nav_msgs::msg::OccupancyGrid &map, int inflation_radius_cm) {
    int width = map.info.width;
    int height = map.info.height;
    int radius_cells = static_cast<int>(std::ceil(inflation_radius_cm / (map.info.resolution * 100.0)));
    std::vector<int8_t> inflated_map = map.data;

    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            if (map.data[y * width + x] == 100)
                for (int dy = -radius_cells; dy <= radius_cells; ++dy)
                    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
                    {
                        int nx = x + dx, ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                            inflated_map[ny * width + nx] = 100;
                    }

    map.data = std::move(inflated_map);
}

int MapLoading::GetCellValue(double x, double y, double z_level) {
    auto it = maps_.find(z_level);
    if (it == maps_.end()) {
        if (maps_.empty()) return -1;
        auto lower = maps_.lower_bound(z_level);
        if (lower == maps_.end()) {
            lower = std::prev(lower);
        }
        z_level = lower->first;
        it = lower;
    }

    const nav_msgs::msg::OccupancyGrid &map = it->second;
    int width = map.info.width;
    int height = map.info.height;
    double res = map.info.resolution;
    int x_idx = static_cast<int>((x - map.info.origin.position.x) / res);
    int y_idx = static_cast<int>((y - map.info.origin.position.y) / res);
    if (x_idx >= 0 && x_idx < width && y_idx >= 0 && y_idx < height)
    {
        int index = y_idx * width + x_idx;
        return map.data[index];
    }
    else
    {
        return -1; // Coordinates out of bounds
    }
}

std::vector<double> MapLoading::GetAvailableZLevels() const
{
    std::vector<double> z_levels;
    for (const auto& pair : maps_) {
        z_levels.push_back(pair.first);
    }
    std::sort(z_levels.begin(), z_levels.end());
    return z_levels;
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