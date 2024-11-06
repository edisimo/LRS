#ifndef PATH_FINDING_HPP
#define PATH_FINDING_HPP
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <tuple>

using Key = std::tuple<int, int, int>;

struct KeyHash {
    std::size_t operator()(const Key& k) const {
        return std::hash<int>()(std::get<0>(k)) ^
               (std::hash<int>()(std::get<1>(k)) << 1) ^
               (std::hash<int>()(std::get<2>(k)) >> 1);
    }
};

struct KeyEqual {
    bool operator()(const Key& lhs, const Key& rhs) const {
        return std::get<0>(lhs) == std::get<0>(rhs) &&
               std::get<1>(lhs) == std::get<1>(rhs) &&
               std::get<2>(lhs) == std::get<2>(rhs);
    }
};

using Map3D = std::unordered_map<Key, int, KeyHash, KeyEqual>;


class MapLoading
{
public:
    MapLoading();
    bool LoadMap(const std::string &map_name, int z_level);
    void PrintMap(int z_level);
    void InflateMap(double inflation_radius_cm, int z_level);
private: 
    const float resolution_ = 0.05;
    int width_;
    int height_;
    Map3D map_data_;
    void InflateCell(Map3D &inflated_map, int x, int y, int z, int inflationRadiusPx);

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
