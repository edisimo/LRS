#include "drone_control/drone_control.hpp"

DroneControl::DroneControl() : Node("drone_control_node")
{
    // Set up ROS publishers, subscribers and service clients
    current_state_.mode = "NONE";
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, std::bind(&DroneControl::StateCallback, this, std::placeholders::_1));
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;
    custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(custom_qos.history, 1), custom_qos);
    local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&DroneControl::LocalPosCallback, this, std::placeholders::_1));

    // Wait for MAVROS SITL connection
    while (rclcpp::ok() && !current_state_.connected)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(100ms);
    }

    ChangeMode("GUIDED");
    
    // TODO: Arm and Take Off 
    RCLCPP_INFO(this->get_logger(), "Sending position command");
    while (!arming_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the arming service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
    }
    

    // TODO: Implement position controller and mission commands here -- mavros setpoint, spravit kruh ci je vramci neho

    // pathfinding bfs priklad 
}

void DroneControl::LocalPosCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    geometry_msgs::msg::PoseStamped current_local_pos_ = *msg;

    // To obtain the position of the drone use this data fields withing the message, please note, that this is the local position of the drone in the NED frame so it is different to the map frame
    // current_local_pos_.pose.position.x
    // current_local_pos_.pose.position.y
    // current_local_pos_.pose.position.z
    // you can do the same for orientation, but you will not need it for this seminar

    RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f", current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
}

void DroneControl::StateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
}


void DroneControl::ChangeMode(std::string mode)
{
     mavros_msgs::srv::SetMode::Request set_mode_request;
    set_mode_request.custom_mode = mode;
    // Wait for GUIDED mode
    
    // TODO: Test if drone state really changed to GUIDED 
    while (!set_mode_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
    }
    bool mode_set = false;
    while(rclcpp::ok() && !mode_set){
        auto result = set_mode_client_->async_send_request(std::make_shared<mavros_msgs::srv::SetMode::Request>(set_mode_request));
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
            if (result.get()->mode_sent){
                RCLCPP_INFO(this->get_logger(), mode + " mode sent, verifying");
                for (int retries = 0; retries < 3; ++retries){
                    std::this_thread::sleep_for(100ms);
                    rclcpp::spin_some(this->get_node_base_interface());

                    if (current_state_.mode == set_mode_request.custom_mode)
                    {
                        mode_set = true;
                        RCLCPP_INFO(this->get_logger(), mode + " mode enabled successfully.");
                        break;
                    }
                }
                if (!mode_set) {
                    RCLCPP_WARN(this->get_logger(), mode + " mode not set after verification attempts. Retrying...");
                }
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Failed to set " + mode + " mode. Retrying...");
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed. Retrying...");
        }
    }
}