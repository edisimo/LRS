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
private: 
    std::vector<Point> points_;
};

class PathFinding : public rclcpp::Node
{
public:
    PathFinding();
private:
    MapLoading map_;
    MissionParser mission_;
};
#endif //PATH_FINDING_HPP
