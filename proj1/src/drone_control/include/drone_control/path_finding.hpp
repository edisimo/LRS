#ifndef PATH_FINDING_HPP
#define PATH_FINDING_HPP
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class MapLoading
{
public:
    MapLoading();
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
