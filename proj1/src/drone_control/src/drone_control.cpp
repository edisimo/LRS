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
    land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;
    custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(custom_qos.history, 1), custom_qos);
    local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&DroneControl::LocalPosCallback, this, std::placeholders::_1));
    path_service_ = this->create_service<drone_control::srv::CustomPath>("custom_path", std::bind(&DroneControl::CustomPathCallback, this, std::placeholders::_1, std::placeholders::_2));
    // Wait for MAVROS SITL connection
    while (rclcpp::ok() && !current_state_.connected)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(100ms);
    }

    TakeOff(2);

    // // pathfinding bfs priklad     
    GoToPoint(0, 0, 2, 0, HARD_THRESHOLD_);
    std::this_thread::sleep_for(1s);
    GoToPoint(0, 0, 2, 90, HARD_THRESHOLD_);
    std::this_thread::sleep_for(1s);
    GoToPoint(0, 0, 2, 180, HARD_THRESHOLD_);
    std::this_thread::sleep_for(1s);
    GoToPoint(0, 0, 2, 270, HARD_THRESHOLD_);
    std::this_thread::sleep_for(1s);
    GoToPoint(0, 0, 2, 0, HARD_THRESHOLD_);

    
    // Land();
}

void DroneControl::CustomPathCallback(const drone_control::srv::CustomPath::Request::SharedPtr request,
                                      drone_control::srv::CustomPath::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Received path request");
    for (const auto &point : request->points)
    {
        RCLCPP_INFO(this->get_logger(), "Point: %f, %f, %f, %s, %s", point.x, point.y, point.z, point.precision.c_str(), point.command.c_str());
    }
    response->success = true;
}

void DroneControl::LocalPosCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_local_pos_ = *msg;
}

void DroneControl::StateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
}


void DroneControl::ChangeMode(std::string mode)
{
    mavros_msgs::srv::SetMode::Request set_mode_request;
    set_mode_request.custom_mode = mode;
    
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
                for (int retries = 0; retries < 5; ++retries){
                    std::this_thread::sleep_for(500ms);
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

void DroneControl::ArmDrone(bool arm_flag)
{
    if (current_state_.mode != "GUIDED")
    {
        RCLCPP_ERROR(this->get_logger(), "Vehicle not in GUIDED mode. Cannot arm.");
        ChangeMode("GUIDED");
    }

    mavros_msgs::srv::CommandBool::Request arm_request;
    arm_request.value = arm_flag;

    RCLCPP_INFO(this->get_logger(), "Sending arming command");
    while (!arming_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the arming service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
    }

    bool armed_status = false;
    for (int retries = 0; retries < 5 && rclcpp::ok(); ++retries)
    {
        auto result = arming_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandBool::Request>(arm_request));
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Arming command sent, verifying");
                std::this_thread::sleep_for(1000ms);
                rclcpp::spin_some(this->get_node_base_interface());

                if (current_state_.armed == arm_flag)
                {
                    armed_status = true;
                    RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully.");
                    break;
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Vehicle not armed after command. Retrying...");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to arm vehicle. Retrying...");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed. Retrying...");
        }
    }

    if (!armed_status)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to arm vehicle after multiple attempts.");
    }
}

void DroneControl::TakeOff(float altitude, float threshold) {
    //TODO: Optional parameter for yaw

    if (current_state_.armed == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Vehicle not armed. Cannot take off.");
        ArmDrone(true);
    }

    tf2::Quaternion q;
    tf2::fromMsg(current_local_pos_.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    mavros_msgs::srv::CommandTOL::Request takeoff_request;
    takeoff_request.min_pitch = 0;
    takeoff_request.yaw = yaw;
    takeoff_request.altitude = altitude;
    //print yaw and altitude
    RCLCPP_INFO(this->get_logger(), "Yaw: %f, Altitude: %f", yaw, altitude);
    while (!takeoff_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the takeoff service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service...");
    }
    auto result = takeoff_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(takeoff_request));
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result.get()->success)
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff sent");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send takeoff");
            return;
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
        return;
    }
    auto start = std::chrono::steady_clock::now();
    while(rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        float dz = altitude - current_local_pos_.pose.position.z;
        float distance = std::sqrt(dz*dz);
        if(distance <= threshold) {
            RCLCPP_INFO(this->get_logger(), "Takeoff position achieved");
            break;
        }
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (elapsed >= TAKEOFF_TIME_LIMIT_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to achieve takeoff position within time limit - %ds", TAKEOFF_TIME_LIMIT_);
            break;
        }
    }
}

void DroneControl::Land() {
    //TODO: Check if the drone is in the air before landing
    //TODO: Verify that the drone has landed
    tf2::Quaternion quat;
    tf2::fromMsg(current_local_pos_.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    mavros_msgs::srv::CommandTOL::Request land_request;
    land_request.yaw = yaw;
    while (!land_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the land service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for land service...");
    }
    auto result = land_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(land_request));
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result.get()->success)
        {
            RCLCPP_INFO(this->get_logger(), "Land sent");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send land");
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }
}

void DroneControl::GoToPoint(float x, float y, float z, float yaw, float threshold) {
    //TODO: Make sure that the drone is in the air actually
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw*M_PI/180.0));
    RCLCPP_INFO(this->get_logger(), "Going to point: %f, %f, %f, %f", x, y, z, yaw); 

    auto timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this, pose]() { 
            PublishPoseCallback(pose);
        }
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    //TODO: Check the yaw
    bool is_within_threshold = false;
    while(rclcpp::ok() && !is_within_threshold) {
        rclcpp::spin_some(this->get_node_base_interface());
        float dx = x - current_local_pos_.pose.position.x;
        float dy = y - current_local_pos_.pose.position.y;
        float dz = z - current_local_pos_.pose.position.z;
        float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        RCLCPP_INFO(this->get_logger(), "Distance: %f", distance);
        is_within_threshold = distance <= threshold;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(this->get_logger(), "Drone at position");
    timer.reset();    
}

void DroneControl::GoToPointGlobal(float x, float y, float z, float yaw, std::string precision, std::string command) {
    std::transform(command.begin(), command.end(), command.begin(), ::toupper);
    std::transform(precision.begin(), precision.end(), precision.begin(), ::toupper);
    const float precision_threshold = precision == "HARD" ? HARD_THRESHOLD_ : SOFT_THRESHOLD_;
    if(command == "TAKEOFF") {
        TakeOff(z);
    }
    GoToPoint(x, y, z, yaw, precision_threshold);
    
}

void DroneControl::PublishPoseCallback(const geometry_msgs::msg::PoseStamped pose) {
    local_pos_pub_->publish(pose);
}

//TODO: FIND OUT WHAT THE HELL IS GOING ON WITH THE ORIENTATION xD  180 - desired_yaw --- not that hard fella :)