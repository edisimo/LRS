#include "drone_control/path_finding.hpp"

PathFinding::PathFinding() : Node("path_finding_node"), astar_(map_)
{   
    path_client_ = this->create_client<drone_control::srv::CustomPath>("custom_path");
    mission_.ParsePoints("/home/lrs-ubuntu/LRS-FEI/mission_1_all.csv");
    // mission_.ParsePoints("/home/lrs-ubuntu/LRS/TEST/test_points.csv");
    mission_.DisplayPoints();
    map_.LoadAllMaps("/home/lrs-ubuntu/LRS-FEI/maps/FEI_LRS_2D/");
    //print all available z levels
    auto z_levels = map_.GetAvailableZLevels();
    for (auto i : z_levels)
    {
        std::cout << i << std::endl;
    }

    // map_.LoadAllMaps("/home/lrs-ubuntu/LRS/TEST/");
    // map_.PrintAllMaps();
    FindTrajectory();
    SendTrajectory();
}

void PathFinding::SendTrajectory() {
    while (!path_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the path service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for path service...");
    }
    auto logger = rclcpp::get_logger("PathFinding");
    auto request = std::make_shared<drone_control::srv::CustomPath::Request>();
    for (const auto& path : trajectory_) {
        for (const auto& point : path.points_) {
            drone_control::msg::CustomPoint point_msg;
            point_msg.x = point.x_;
            point_msg.y = point.y_;
            point_msg.z = point.z_;
            point_msg.precision = point.precision_;
            point_msg.command = point.command_;
            request->points.push_back(point_msg);
        }
    }
    auto result = path_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result.get()->success)
        {
            RCLCPP_INFO(logger, "Path sent");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to send path");
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Service call failed");
    }
}

void PathFinding::FindTrajectory()
{
    RCLCPP_INFO(this->get_logger(), "Running path finding...");
    std::vector<Point>& points = mission_.GetPoints();
    auto z_levels = map_.GetAvailableZLevels();
    if (points.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Not enough mission points.");
        return;
    }
    int i = 0;

    while(points.size() > 1) {
        Point& start = points[0];
        Point& goal = points[1];
        std::vector<astar::Node> path = astar_.FindPath(start.x_, start.y_, start.z_,
                                goal.x_, goal.y_, goal.z_);
        if (!path.empty()) {
            std::cout << "Path found:" << std::endl;
        } 
        else {
            std::cout << "No path found between start and goal." << std::endl;
        }
        path = SimplifyPath(path, map_);
        RCLCPP_INFO(this->get_logger(), "Path found with %lu nodes.", path.size());
        // map_.PrintAllMaps(path);
        ActualPath actual_path;
        auto start_path_size = path.size();
        while (!path.empty()) {
            auto node = path.front();
            path.erase(path.begin());
            //first point
            if (path.size()==start_path_size-1 && i==0) {
                actual_path.AddPoint(Point(node.x_*map_.GetResolution(), node.y_*map_.GetResolution(), z_levels[node.z_], start.precision_, start.command_));
            }   
            //last point
            else if (path.empty()) {
                actual_path.AddPoint(Point(node.x_*map_.GetResolution(), node.y_*map_.GetResolution(), z_levels[node.z_], goal.precision_, goal.command_));
            }
            else {
                actual_path.AddPoint(Point(node.x_*map_.GetResolution(), node.y_*map_.GetResolution(), z_levels[node.z_], "soft", "-"));
            }
        }
        trajectory_.push_back(actual_path);
        points.erase(points.begin());
        i++;
    }
    RCLCPP_INFO(this->get_logger(), "Finished path finding. Trajectory size: %lu", trajectory_.size());
    for (const auto& path : trajectory_) {
        RCLCPP_INFO(this->get_logger(), "Actual path:");
        for (const auto& point : path.points_) {
            RCLCPP_INFO(this->get_logger(), "Point: (%f, %f, %f), Precision: %s, Command: %s", point.x_, point.y_, point.z_, point.precision_.c_str(), point.command_.c_str());
        }
    }
}

std::vector<astar::Node> PathFinding::SimplifyPath(const std::vector<astar::Node>& path, MapLoading& map) {
    if (path.empty()) return {};

    std::vector<astar::Node> simplified_path;
    simplified_path.push_back(path.front()); 

    size_t current_index = 0;

    while (current_index < path.size() - 1) {
        size_t next_index = path.size() - 1; 

        
        while (next_index > current_index + 1) {
            if (IsLineOfSight(path[current_index], path[next_index], map)) {
                break; 
            }
            --next_index;
        }

        
        simplified_path.push_back(path[next_index]);
        current_index = next_index;
    }

    return simplified_path;
}

bool PathFinding::IsLineOfSight(const astar::Node& start, const astar::Node& end, MapLoading& map) {
    auto logger = rclcpp::get_logger("IsLineOfSight");
    int x0 = start.x_;
    int y0 = start.y_;
    int z0 = start.z_;
    int x1 = end.x_;
    int y1 = end.y_;
    int z1 = end.z_;

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dz = abs(z1 - z0);

    int xs = (x1 > x0) ? 1 : -1;
    int ys = (y1 > y0) ? 1 : -1;
    int zs = (z1 > z0) ? 1 : -1;

    //X
    if (dx >= dy && dx >= dz) {
        int p1 = 2 * dy - dx;
        int p2 = 2 * dz - dx;

        while (x0 != x1) {
            x0 += xs;

            if (p1 >= 0) {
                y0 += ys;
                p1 -= 2 * dx;
            }
            if (p2 >= 0) {
                z0 += zs;
                p2 -= 2 * dx;
            }

            p1 += 2 * dy;
            p2 += 2 * dz;

            if (map.GetCellValue(x0 * MapLoading::GetResolution(), y0 * MapLoading::GetResolution(), map.GetAvailableZLevels()[z0]) == 100) {
                return false;
            }
        }
    }
    //Y
    else if (dy >= dx && dy >= dz) {
        int p1 = 2 * dx - dy;
        int p2 = 2 * dz - dy;

        while (y0 != y1) {
            y0 += ys;

            if (p1 >= 0) {
                x0 += xs;
                p1 -= 2 * dy;
            }
            if (p2 >= 0) {
                z0 += zs;
                p2 -= 2 * dy;
            }

            p1 += 2 * dx;
            p2 += 2 * dz;

            if (map.GetCellValue(x0 * MapLoading::GetResolution(), y0 * MapLoading::GetResolution(), map.GetAvailableZLevels()[z0]) == 100) {
                return false;
            }
        }
    }
    //Z
    else {
        int p1 = 2 * dy - dz;
        int p2 = 2 * dx - dz;

        while (z0 != z1) {
            z0 += zs;

            if (p1 >= 0) {
                y0 += ys;
                p1 -= 2 * dz;
            }
            if (p2 >= 0) {
                x0 += xs;
                p2 -= 2 * dz;
            }

            p1 += 2 * dy;
            p2 += 2 * dx;

            if (map.GetCellValue(x0 * MapLoading::GetResolution(), y0 * MapLoading::GetResolution(), map.GetAvailableZLevels()[z0]) == 100) {
                return false;
            }
        }
    }

    return true;
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
        std::cout << "Z-level: " << z_level << std::endl;
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
    RCLCPP_INFO(logger, "Map at z-level %f:", z_level); 

    const nav_msgs::msg::OccupancyGrid &map = it->second;
    int width = map.info.width;
    int height = map.info.height;

    auto z_levels = GetAvailableZLevels();

    std::set<std::pair<int, int>> path_coords;
    for (const auto& node : path) {
        auto node_z = z_levels[node.z_];
        if (fabs(node_z - z_level) < 1e-3) { 
            path_coords.insert({node.x_, node.y_});
        }
    }

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

int MapLoading::GetZIndex(double z) const {
    auto z_levels = GetAvailableZLevels();
    if (z_levels.empty()) return -1;

    auto lower = std::lower_bound(z_levels.begin(), z_levels.end(), z);
    if (lower == z_levels.end()) {
        return z_levels.size() - 1;
    }
    if (lower == z_levels.begin()) {
        return 0;
    }
    auto prev = std::prev(lower);
    if (std::abs(*prev - z) < std::abs(*lower - z)) {
        return std::distance(z_levels.begin(), prev);
    } else {
        return std::distance(z_levels.begin(), lower);
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
    auto logger = rclcpp::get_logger("AStar3D");

    const double resolution = MapLoading::GetResolution();

    int start_x_idx = static_cast<int>(std::round(start_x / resolution));
    int start_y_idx = static_cast<int>(std::round(start_y / resolution));
    int start_z_idx = map_.GetZIndex(start_z);

    int goal_x_idx = static_cast<int>(std::round(goal_x / resolution));
    int goal_y_idx = static_cast<int>(std::round(goal_y / resolution));
    int goal_z_idx = map_.GetZIndex(goal_z);

    if (start_z_idx == -1 || goal_z_idx == -1) {
        RCLCPP_ERROR(logger, "Invalid start or goal z-level.");
        return {};
    }
    auto z_levels = map_.GetAvailableZLevels();

    std::priority_queue<astar::Node, std::vector<astar::Node>, std::greater<astar::Node>> open_list;
    std::unordered_set<std::tuple<int, int, int>, HashFunc> closed_set;
    
    RCLCPP_INFO(logger, "Finding path from (%f, %f, %f) to (%f, %f, %f)", start_x, start_y, start_z, goal_x, goal_y, goal_z);
    auto start_node = std::make_shared<astar::Node>(start_x_idx, start_y_idx, start_z_idx, 0.0, 0.0, nullptr);
    start_node->h_ = std::sqrt(
        std::pow((goal_x_idx - start_x_idx) * resolution, 2) +
        std::pow((goal_y_idx - start_y_idx) * resolution, 2) +
        std::pow(z_levels[goal_z_idx] - z_levels[start_z_idx], 2)
    );
    start_node->f_ = start_node->g_ + start_node->h_;
    open_list.push(*start_node);

    std::vector<std::tuple<int, int, int>> directions;
    for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz)
                if (dx != 0 || dy != 0 || dz != 0)
                    directions.emplace_back(dx, dy, dz);

    RCLCPP_INFO(logger, "Resolution: %f", resolution);

    while (!open_list.empty() && rclcpp::ok()) {
        astar::Node current = open_list.top();
        open_list.pop();
        std::cout << "Open list size " << open_list.size() << std::endl;

        if (current.x_ == goal_x_idx && current.y_ == goal_y_idx && current.z_ == goal_z_idx) {
            RCLCPP_INFO(logger, "Goal reached.");
            std::vector<astar::Node> path;
            std::shared_ptr<astar::Node> node = std::make_shared<astar::Node>(current);
            while (node != nullptr) {
                path.emplace_back(astar::Node{node->x_, node->y_, node->z_, node->g_, node->h_, node->parent_});
                node = node->parent_;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        closed_set.insert(std::make_tuple(current.x_, current.y_, current.z_));

        for (const auto& dir : directions) {
            int nx = current.x_ + std::get<0>(dir);
            int ny = current.y_ + std::get<1>(dir);
            int nz = current.z_ + std::get<2>(dir);

            if (nz < 0 || nz >= static_cast<int>(z_levels.size())) continue;

            auto neighbor_key = std::make_tuple(nx, ny, nz);
            if (closed_set.find(neighbor_key) != closed_set.end()) continue;

            double wx = nx * resolution;
            double wy = ny * resolution;
            double wz = z_levels[nz];

            int cell_value = map_.GetCellValue(wx, wy, wz);
            if (cell_value == 100 || cell_value == -1) continue; 

            double movement_cost = std::sqrt(
                std::pow(std::get<0>(dir) * resolution, 2) +
                std::pow(std::get<1>(dir) * resolution, 2) +
                std::pow(z_levels[nz] - z_levels[current.z_], 2)
            );
            double tentative_g = current.g_ + movement_cost;
            double h = std::sqrt(
                std::pow((goal_x_idx - nx) * resolution, 2) +
                std::pow((goal_y_idx - ny) * resolution, 2) +
                std::pow(z_levels[goal_z_idx] - z_levels[nz], 2)
            );

            auto neighbor = std::make_shared<astar::Node>(nx, ny, nz, tentative_g, h, std::make_shared<astar::Node>(current));
            open_list.push(*neighbor);
        }
    }

    return {};
}