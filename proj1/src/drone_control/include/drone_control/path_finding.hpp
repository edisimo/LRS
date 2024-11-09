#ifndef PATH_FINDING_HPP
#define PATH_FINDING_HPP
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <tuple>
#include <filesystem>
#include <iomanip>
#include <queue>
#include <cmath>
#include <map>
#include <unordered_set>

namespace astar {
    struct Node {
        // Not a ros node, but A* node
        int x_, y_, z_;
        double g_, h_, f_;
        std::shared_ptr<Node> parent_;
        // Default constructor necessary for .... some reason i guess something with the unordered map
        Node()
            : x_(0), y_(0), z_(0), g_(0), h_(0), f_(0), parent_(nullptr) {}

        Node(int x, int y, int z, double g, double h, std::shared_ptr<Node> parent)
            : x_(x), y_(y), z_(z), g_(g), h_(h), f_(g + h), parent_(parent) {}

        bool operator>(const Node& other) const {
            return f_ > other.f_;
        }
};

}

struct HashFunc {
    size_t operator()(const std::tuple<int, int, int>& key) const {
        auto [x, y, z] = key;
        return std::hash<int>()(x) ^ std::hash<int>()(y) << 1 ^ std::hash<int>()(z) << 2;
    }
};


class MapLoading
{
public:
    MapLoading();
    void LoadAllMaps(const std::string &directory_path);
    bool LoadMapPGM(const std::string &map_name, nav_msgs::msg::OccupancyGrid &map, double z_level);
    int GetCellValue(double x, double y, double z_level);
    std::vector<double> GetAvailableZLevels() const;
    constexpr static double GetResolution() { return resolution_; }
    void PrintAllMaps();
    void PrintMap(double z_level);
    void PrintMap(double z_level, const std::vector<astar::Node>& path);
    void PrintAllMaps(const std::vector<astar::Node>& path);
    int GetZIndex(double z) const;
private: 
    static constexpr double resolution_ = 0.05;
    // static constexpr float inflation_radius_cm_ = 25.0;
    static constexpr double inflation_radius_cm_ = 0.0;
    std::map<double, nav_msgs::msg::OccupancyGrid> maps_;
    void InflateObstacles(nav_msgs::msg::OccupancyGrid &map, int inflation_radius_cm);
};

struct Point {
    double x_;
    double y_;
    double z_;
    std::string precision_;
    std::string command_;
    Point(double x, double y, double z, const std::string& precision, const std::string& command)
        : x_(x), y_(y), z_(z), precision_(precision), command_(command) {}
};

class MissionParser
{
public:
    MissionParser();
    void ParsePoints(const std::string& filename);
    void DisplayPoints();
    const std::vector<Point>& GetPoints() const { return points_; };
private: 
    std::vector<Point> points_;
};

class AStar3D
{
public:
    AStar3D(MapLoading& map_loader);
    std::vector<astar::Node> FindPath(double start_x, double start_y, double start_z,
                               double goal_x, double goal_y, double goal_z);
private:
    MapLoading& map_;
};

class PathFinding : public rclcpp::Node
{
public:
    PathFinding();
    void Run();
private:
    MapLoading map_;
    MissionParser mission_;
    AStar3D astar_;
};
#endif //PATH_FINDING_HPP
