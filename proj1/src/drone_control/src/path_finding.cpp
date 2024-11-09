#include "drone_control/path_finding.hpp"

PathFinding::PathFinding() : Node("path_finding_node"), astar_(map_)
{
    mission_.ParsePoints("/home/lrs-ubuntu/LRS-FEI/mission_1_all.csv");
    mission_.DisplayPoints();
    // map_.LoadAllMaps("/home/lrs-ubuntu/LRS-FEI/maps/FEI_LRS_2D/");
    map_.LoadAllMaps("/home/lrs-ubuntu/LRS/TEST/");
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "After inflation:" << std::endl;
    map_.PrintAllMaps();

    int a = map_.GetCellValue(0.01, 0.45, 0.3);
    int b = map_.GetCellValue(0.04, 0.45, 0.3);
    int c = map_.GetCellValue(0.06, 0.45, 0.3);
    int d = map_.GetCellValue(0.09, 0.45, 0.3);
    int e = map_.GetCellValue(0.44, 0.45, 0.3);
    int f = map_.GetCellValue(0.46, 0.45, 0.3);
    std::cout << "Cell1: " << a << std::endl;
    std::cout << "Cell2: " << b << std::endl;
    std::cout << "Cell3: " << c << std::endl;
    std::cout << "Cell4: " << d << std::endl;
    std::cout << "Cell5: " << e << std::endl;
    std::cout << "Cell6: " << f << std::endl;
}

void PathFinding::Run()
{
    // Get the mission points
    const std::vector<Point>& points = mission_.GetPoints();

    // Assuming we have at least two points
    if (points.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Not enough mission points.");
        return;
    }

    const Point& start = points[0];
    const Point& goal = points[1];

    // Find path using AStar3D
    std::vector<astar::Node> path = astar_.FindPath(start.x_, start.y_, start.z_,
                                             goal.x_, goal.y_, goal.z_);

    if (!path.empty()) {
        std::cout << "Path found:" << std::endl;
        for (const auto& node : path) {
            std::cout << "(" << node.x_ << ", " << node.y_ << ", " << node.z_ << ")" << std::endl;
        }

        // Print maps with the path marked
        map_.PrintAllMaps(path);
    } else {
        std::cout << "No path found between start and goal." << std::endl;
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
    InflateObstacles(map, inflation_radius_cm_);
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
    auto logger = rclcpp::get_logger("MapLoading");
    auto available_z_levels = GetAvailableZLevels();
    std::sort(available_z_levels.begin(), available_z_levels.end());
    //print levels
    // for (auto i : available_z_levels)
    // {
    //     std::cout << i << std::endl;
    // }
    auto lower = std::lower_bound(available_z_levels.begin(), available_z_levels.end(), z_level);
    if (lower == available_z_levels.end()) {
        lower = std::prev(lower);
    } else if (lower != available_z_levels.begin()) {
        auto prev = std::prev(lower);
        if (std::abs(*prev - z_level) < std::abs(*lower - z_level)) {
            lower = prev;
        }
    }
    z_level = *lower;

    auto it = maps_.find(z_level);
    if (it == maps_.end()) {
        RCLCPP_ERROR(logger, "Map at z-level %f not found.", z_level);
        return -1;
    }

    const nav_msgs::msg::OccupancyGrid &map = it->second;
    int width = map.info.width;
    int height = map.info.height;
    double res = map.info.resolution;

    int x_idx = static_cast<int>(std::round((x - map.info.origin.position.x) / res));
    int y_idx = static_cast<int>(std::round((y - map.info.origin.position.y) / res));

    if (x_idx >= 0 && x_idx < width && y_idx >= 0 && y_idx < height)
    {
        int index = y_idx * width + x_idx;
        return map.data[index];
    }
    else
    {
        return -1;
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

void MapLoading::PrintMap(double z_level)
{   
    auto logger = rclcpp::get_logger("MapLoading");
    auto it = maps_.find(z_level);
    if (it == maps_.end()) {
        RCLCPP_ERROR(logger, "Map at z-level %f not found.", z_level);
        return;
    }

    const nav_msgs::msg::OccupancyGrid &map = it->second;
    int width = map.info.width;
    int height = map.info.height;

    RCLCPP_INFO(logger, "Map at z-level %f:", z_level);
    

    for (int y = height - 1; y >= 0; --y)
    {
        for (int x = 0; x < width; ++x)
        {
            int index = y * width + x;
            int8_t value = map.data[index];
            char display_char;

            if (value == 100)
                display_char = '#'; // Occupied
            else if (value == 0)
                display_char = '.'; // Free
            else
                display_char = ' '; // Unknown

            std::cout << display_char;
        }
        std::cout << std::endl;
    }
}

void MapLoading::PrintAllMaps()
{
    for (const auto &pair : maps_)
    {
        double z_level = pair.first;
        PrintMap(z_level);
        std::cout << std::endl;
    }
}

void MapLoading::PrintMap(double z_level, const std::vector<astar::Node>& path)
{
    auto logger = rclcpp::get_logger("MapLoading");
    auto it = maps_.find(z_level);
    if (it == maps_.end()) {
        RCLCPP_ERROR(logger, "Map at z-level %f not found.", z_level);
        return;
    }

    const nav_msgs::msg::OccupancyGrid &map = it->second;
    int width = map.info.width;
    int height = map.info.height;
    double res = map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;

    // Create a set of path coordinates for quick lookup
    std::set<std::pair<int, int>> path_coords;
    for (const auto& node : path) {
        if (fabs(node.z_ - z_level) < 1e-3) { // Nodes at this z-level
            int x_idx = static_cast<int>((node.x_ - origin_x) / res);
            int y_idx = static_cast<int>((node.y_ - origin_y) / res);
            path_coords.insert({x_idx, y_idx});
        }
    }

    RCLCPP_INFO(logger, "Map at z-level %f:", z_level);

    for (int y = height - 1; y >= 0; --y)
    {
        for (int x = 0; x < width; ++x)
        {
            int index = y * width + x;
            int8_t value = map.data[index];
            char display_char;

            if (path_coords.find({x, y}) != path_coords.end())
                display_char = '*'; // Path
            else if (value == 100)
                display_char = '#'; // Occupied
            else if (value == 0)
                display_char = '.'; // Free
            else
                display_char = ' '; // Unknown

            std::cout << display_char;
        }
        std::cout << std::endl;
    }
}

void MapLoading::PrintAllMaps(const std::vector<astar::Node>& path)
{
    for (const auto &pair : maps_)
    {
        double z_level = pair.first;
        PrintMap(z_level, path);
        std::cout << std::endl;
    }
}


MissionParser::MissionParser()
{
}

void MissionParser::ParsePoints(const std::string& filename) {
    auto logger = rclcpp::get_logger("MissionParser");
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
            RCLCPP_ERROR(logger, "Failed to parse file");
            return;
        }
    }
    std::cout << "Finished" << std::endl;
}

void MissionParser::DisplayPoints() {
    for (const auto& point : points_) {        
        RCLCPP_INFO(rclcpp::get_logger("MissionParser"), "Point: (%f, %f, %f), Precision: %s, Command: %s", point.x_, point.y_, point.z_, point.precision_.c_str(), point.command_.c_str());
    }
}

AStar3D::AStar3D(MapLoading& map_loader)
    : map_(map_loader)
{
}

std::vector<astar::Node> AStar3D::FindPath(double start_x, double start_y, double start_z,
                                    double goal_x, double goal_y, double goal_z)
{
    // Open list (priority queue) and closed set
    std::priority_queue<astar::Node, std::vector<astar::Node>, std::greater<astar::Node>> open_list;
    std::unordered_map<std::string, astar::Node> closed_set;

    // Start node
    astar::Node start_node(start_x, start_y, start_z, 0.0, 0.0, nullptr);
    start_node.h_ = sqrt(pow(goal_x - start_x, 2) +
                        pow(goal_y - start_y, 2) +
                        pow(goal_z - start_z, 2));
    start_node.f_ = start_node.g_ + start_node.h_;
    open_list.push(start_node);

    // Directions for neighbors in 3D (26 directions including diagonals)
    std::vector<std::tuple<int, int, int>> directions;
    for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz)
                if (dx != 0 || dy != 0 || dz != 0)
                    directions.emplace_back(dx, dy, dz);

    const double resolution = MapLoading::GetResolution();

    while (!open_list.empty()) {
        astar::Node current = open_list.top();
        open_list.pop();

        // Check if goal is reached
        if (fabs(current.x_ - goal_x) < 1e-6 && fabs(current.y_ - goal_y) < 1e-6 && fabs(current.z_ - goal_z) < 1e-6) {
            // Reconstruct path
            std::vector<astar::Node> path;
            astar::Node* node = &current;
            while (node != nullptr) {
                path.push_back(*node);
                node = node->parent_;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Mark current node as visited
        std::string current_key = std::to_string(current.x_) + "_" + std::to_string(current.y_) + "_" + std::to_string(current.z_);
        closed_set[current_key] = current;

        // Explore neighbors
        for (const auto& dir : directions) {
            double nx = current.x_ + std::get<0>(dir) * resolution;
            double ny = current.y_ + std::get<1>(dir) * resolution;
            double nz = current.z_ + std::get<2>(dir) * resolution;

            // Check if neighbor is in closed set
            std::string neighbor_key = std::to_string(nx) + "_" + std::to_string(ny) + "_" + std::to_string(nz);
            if (closed_set.find(neighbor_key) != closed_set.end()) continue;

            // Check if the cell is walkable
            int cell_value = map_.GetCellValue(nx, ny, nz);
            if (cell_value == 100 || cell_value == -1) continue; // Obstacle or unknown

            // Compute costs
            double movement_cost = sqrt(pow(std::get<0>(dir), 2) +
                                        pow(std::get<1>(dir), 2) +
                                        pow(std::get<2>(dir), 2)) * resolution;
            double tentative_g = current.g_ + movement_cost;
            double h = sqrt(pow(goal_x - nx, 2) +
                            pow(goal_y - ny, 2) +
                            pow(goal_z - nz, 2));

            astar::Node neighbor(nx, ny, nz, tentative_g, h, new astar::Node(current));

            open_list.push(neighbor);
        }
    }

    return std::vector<astar::Node>();
}