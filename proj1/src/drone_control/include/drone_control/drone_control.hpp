#ifndef DRONE_CONTROL_HPP
#define DRONE_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <chrono>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/convert.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "drone_control/srv/custom_path.hpp"
#include "drone_control/msg/custom_point.hpp"


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
    void TakeOff(float altitude, float threshold = SOFT_THRESHOLD_);
    void Land();
    void LandTakeoff();
    void GoToPoint(float x, float y, float z, float yaw, float threshold);
    void GoToPointGlobal(float x, float y, float z, float yaw, std::string precision , std::string command);
    void CustomPathCallback(const drone_control::srv::CustomPath::Request::SharedPtr request,
                      drone_control::srv::CustomPath::Response::SharedPtr response);

    static constexpr float SOFT_THRESHOLD_ = 0.1f;
    static constexpr float HARD_THRESHOLD_ = 0.05;
    static constexpr int TAKEOFF_TIME_LIMIT_ = 15; 

    double drone_x_global_, drone_y_global_, drone_z_global_, drone_yaw_global_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    rclcpp::Service<drone_control::srv::CustomPath>:: SharedPtr path_service_;
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_local_pos_;

};

#endif //DRONE_CONTROL_HPP
