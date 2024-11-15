#ifndef PATH_FINDING_HPP
#define PATH_FINDING_HPP
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <iostream>
#include <chrono>
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
#include "drone_control/srv/custom_path.hpp"
#include "drone_control/msg/custom_point.hpp"

using namespace std::chrono_literals;

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

        bool operator<(const Node& other) const {
            return f_ < other.f_;
        }
        bool operator>(const Node& other) const {
            return f_ > other.f_;
        }
};

}


struct HashFunc {
    std::size_t operator()(const std::tuple<int, int, int>& key) const {
        int x = std::get<0>(key);
        int y = std::get<1>(key);
        int z = std::get<2>(key);
        return ((std::hash<int>()(x) ^ (std::hash<int>()(y) << 1)) >> 1) ^ (std::hash<int>()(z) << 1);
    }
};

class MapLoading
{
public:
    MapLoading();
    void LoadAllMaps(const std::string &directory_path);
    bool LoadMapPGM(const std::string &map_name, nav_msgs::msg::OccupancyGrid &map, double z_level);
    bool SaveMapPGM(const std::string &map_name, const nav_msgs::msg::OccupancyGrid &map);
    int GetCellValue(double x, double y, double z_level);
    std::vector<double> GetAvailableZLevels() const;
    constexpr static double GetResolution() { return RESOLUTION_; }
    void PrintAllMaps();
    void PrintMap(double z_level);
    void PrintMap(double z_level, const std::vector<astar::Node>& path);
    void PrintAllMaps(const std::vector<astar::Node>& path);
    int GetZIndex(double z) const;
    int GetWidth() const;
    int GetHeight() const;
private: 
    static constexpr double RESOLUTION_ = 0.05;
    static constexpr float INFLATION_RADIUS_CM_ = 30.0;
    static constexpr float Y_OFFSET_ = 1.0;
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

class ActualPath {
    public:
        ActualPath(){};
        void AddPoint(const Point& point) { points_.push_back(point); }
        std::vector<Point> points_;
};

class MissionParser
{
public:
    MissionParser();
    void ParsePoints(const std::string& filename);
    void DisplayPoints();
    std::vector<Point>& GetPoints() { return points_; };
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
    static constexpr float HEURISTIC_WEIGHT_ = 7.0;
    static constexpr float CUMULATIVE_WEIGHT_ = 1.0;
};

class PathFinding : public rclcpp::Node
{
public:
    PathFinding();
    void FindTrajectory();
    void SendTrajectory();
    std::vector<astar::Node> SimplifyPath(const std::vector<astar::Node>& path, MapLoading& map);
    bool IsLineOfSight(const astar::Node& start, const astar::Node& end, MapLoading& map); 
private:
    MapLoading map_;
    MissionParser mission_;
    AStar3D astar_;
    std::vector<ActualPath> trajectory_;
    rclcpp::Client<drone_control::srv::CustomPath>::SharedPtr  path_client_;   
};
#endif //PATH_FINDING_HPP
