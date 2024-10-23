#ifndef DRONE_CONTROL_HPP
#define DRONE_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl();

private:
    void LocalPosCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void StateCallback(const mavros_msgs::msg::State::SharedPtr msg);
    void PublishPoseCallback(const geometry_msgs::msg::PoseStamped pose);
    void ChangeMode(std::string mode);
    void ArmDrone(bool arm_flag);
    void TakeOff(float min_pitch, float yaw, float altitude, float threshold);
    void Land(float min_pitch, float yaw, float altitude);
    void GoToPoint(float x, float y, float z, float yaw, float threshold);

    static constexpr float SOFT_THRESHOLD_ = 0.1f;
    static constexpr float HARD_THRESHOLD_ = 0.05;
    static constexpr int TAKEOFF_TIME_LIMIT_ = 15; 


    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_local_pos_;
};

#endif //DRONE_CONTROL_HPP
