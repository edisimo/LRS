#include "drone_control/drone_control.hpp"

DroneControl::DroneControl() : Node("drone_control_node")
{
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
    while (rclcpp::ok() && !current_state_.connected)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(100ms);
    }
    while (rclcpp::ok() && !received_mission_)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(100ms);
    }
    if (received_mission_)
    {
        for (const auto &point : mission_->points)
        {
            GoToPointGlobal(point.x, point.y, point.z, point.precision, point.command);
        }            
    }
          
}

void DroneControl::CustomPathCallback(const drone_control::srv::CustomPath::Request::SharedPtr request,
                                      drone_control::srv::CustomPath::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Received path request");
    mission_ = request;
    received_mission_ = true;
    response->success = true;
}

void DroneControl::LocalPosCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_local_pos_ = *msg;
    drone_x_local_ = current_local_pos_.pose.position.x;
    drone_y_local_ = current_local_pos_.pose.position.y;
    drone_z_local_ = current_local_pos_.pose.position.z;
    tf2::Quaternion q;
    tf2::fromMsg(current_local_pos_.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    drone_yaw_local_ = yaw;

    drone_x_global_ = LocalToGlobalX(drone_y_local_);
    drone_y_global_ = LocalToGlobalY(drone_x_local_);
    drone_z_global_ = drone_z_local_;
    drone_yaw_global_ = LocalToGlobalYaw(drone_yaw_local_);

}

void DroneControl::StateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state_ = *msg;
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
                for (int retries = 0; retries < MAX_RETRIES_; ++retries){
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
        RCLCPP_WARN(this->get_logger(), "Vehicle not in GUIDED mode. Cannot arm. Trying to change mode...");
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
    for (int retries = 0; retries < MAX_RETRIES_ && rclcpp::ok(); ++retries)
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
    if (current_state_.armed == false)
    {
        RCLCPP_WARN(this->get_logger(), "Vehicle not armed. Cannot take off. Arming....");
        ArmDrone(true);
    }

    mavros_msgs::srv::CommandTOL::Request takeoff_request;
    takeoff_request.min_pitch = 0;
    takeoff_request.yaw = drone_yaw_local_;
    takeoff_request.altitude = altitude;
    RCLCPP_INFO(this->get_logger(), "Yaw: %f, Altitude: %f", drone_yaw_local_, altitude);
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
        float dz = altitude - drone_z_local_;
        float distance = std::sqrt(dz*dz);
        if(distance <= threshold) {
            RCLCPP_INFO(this->get_logger(), "Takeoff position achieved");
            break;
        }
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (elapsed >= LAND_TAKEOFF_TIME_LIMIT_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to achieve takeoff position within time limit - %ds", LAND_TAKEOFF_TIME_LIMIT_);
            break;
        }
    }
}

void DroneControl::Land() {
    mavros_msgs::srv::CommandTOL::Request land_request;
    land_request.yaw = drone_yaw_local_;
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
    auto start = std::chrono::steady_clock::now();
    while(rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        float dz = 0 - drone_z_local_;
        float distance = std::sqrt(dz*dz);
        if(distance <= SOFT_THRESHOLD_) {
            RCLCPP_INFO(this->get_logger(), "Land position achieved");
            break;
        }
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (elapsed >= LAND_TAKEOFF_TIME_LIMIT_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to achieve land position within time limit - %ds", LAND_TAKEOFF_TIME_LIMIT_);
            break;
        }
    }
}

void DroneControl::LandTakeoff(double altitude, float threshold) {
    Land();
    while(rclcpp::ok() && current_state_.armed) {
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    TakeOff(altitude, threshold);
}

void DroneControl::GoToPoint(float x, float y, float z, float yaw, float threshold, float yaw_threshold) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw*M_PI/180.0));
    RCLCPP_INFO(this->get_logger(), "Going to point: %f, %f, %f, %f", LocalToGlobalX(y), LocalToGlobalY(x), z, yaw*M_PI/180.0); 

    auto timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this, pose]() { 
            PublishPoseCallback(pose);
        }
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    bool is_within_threshold = false;
    bool is_yaw_within_threshold = false;
    while(rclcpp::ok() && (!is_within_threshold || !is_yaw_within_threshold)) {
        rclcpp::spin_some(this->get_node_base_interface());
        float dx = x - drone_x_local_;
        float dy = y - drone_y_local_;
        float dz = z - drone_z_local_;
        float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        float dyaw = fabs(LocalToGlobalYaw(yaw*M_PI/180.0) - drone_yaw_global_);
        is_within_threshold = distance <= threshold;
        is_yaw_within_threshold = dyaw <= yaw_threshold;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(this->get_logger(), "Drone at point (LOCAL): %f, %f, %f, %f", drone_x_local_, drone_y_local_, drone_z_local_, drone_yaw_local_);
    timer.reset();    
}

void DroneControl::GoToPointGlobal(float x, float y, float z, std::string precision, std::string command) {
    std::transform(command.begin(), command.end(), command.begin(), ::toupper);
    std::transform(precision.begin(), precision.end(), precision.begin(), ::toupper);
    const float precision_threshold = precision == "HARD" ? HARD_THRESHOLD_ : SOFT_THRESHOLD_;
    const float yaw_precision_threshold = precision == "HARD" ? HARD_YAW_THRESHOLD_ : SOFT_YAW_THRESHOLD_;
    double yaw = drone_yaw_global_;
    if (command.find("YAW") == 0) { 
        std::string number_part = command.substr(3); 

        int yaw_value = std::stoi(number_part);
        yaw = yaw_value*M_PI/180.0;
    }
    else if(command == "TAKEOFF") {
        TakeOff(z, precision_threshold);
    }
    GoToPoint(GlobalToLocalX(y), GlobalToLocalY(x), z, 180.0*GlobalToLocalYaw(yaw)/M_PI, precision_threshold, yaw_precision_threshold);
    RCLCPP_INFO(this->get_logger(), "Drone at point (GLOBAL): %f, %f, %f, %f", drone_x_global_, drone_y_global_, drone_z_global_, drone_yaw_global_*180.0/M_PI);
    if(command == "LAND") {
        Land();
    }
    else if(command == "LANDTAKEOFF") {
        LandTakeoff(z, precision_threshold);
    }
    
}

void DroneControl::PublishPoseCallback(const geometry_msgs::msg::PoseStamped pose) {
    local_pos_pub_->publish(pose);
}
