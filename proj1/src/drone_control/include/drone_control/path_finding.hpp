#ifndef PATH_FINDING_HPP
#define PATH_FINDING_HPP
#include <rclcpp/rclcpp.hpp>

class MapLoading
{
public:
    MapLoading();
};

class MissionParser
{
public:
    MissionParser();
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
