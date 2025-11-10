#include "bt_aruco_landing/bt_nodes/go_to_GPS.hpp"

namespace bt_aruco_landing
{
GoToGPS::GoToGPS(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config){
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/simple_drone/msdk_cmd_vel", 10);
    // Use reliable QoS for attitude data (matches publisher)
    attitude_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/dji_msdk_ros/drone_attitude",
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable),
        std::bind(&GoToGPS::attitudeCallback, this, std::placeholders::_1));
    // Use sensor QoS for GPS position data  
    position_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/dji_msdk_ros/gps_position", rclcpp::SensorDataQoS(), std::bind(&GoToGPS::positionCallback, this, std::placeholders::_1)); 
}

BT::NodeStatus GoToGPS::tick()
{
    auto gps_input = this->getInput<sensor_msgs::msg::NavSatFix>("gps_goal");
    bool landing_init = this->getInput<bool>("landing_init").value_or(false);
    if (!gps_input) {
        RCLCPP_WARN(node_->get_logger(), "Missing GPS goal");
        return BT::NodeStatus::FAILURE;
    }

    current_gps_ = std::make_shared<sensor_msgs::msg::NavSatFix>(gps_input.value());
    bool twisting = true;
    while (true)
    {
        // Spin to get latest data without holding the lock
        rclcpp::spin_some(node_);
        
        // Check if we have required data
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!current_position_ || !current_attitude_) {
                RCLCPP_WARN(node_->get_logger(), "Missing DJI position or attitude data");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }
        
        // Copy data to local variables to avoid holding lock during calculations
        sensor_msgs::msg::NavSatFix position_copy;
        geometry_msgs::msg::Vector3 attitude_copy;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            position_copy = *current_position_;
            attitude_copy = *current_attitude_;
        }
        
        float bearing = atan2(current_gps_->longitude - position_copy.longitude,
            current_gps_->latitude - position_copy.latitude);
        float drone_yaw = attitude_copy.z * M_PI / 180.0;
        float bearing_error = bearing - drone_yaw;
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0; // Maintain current altitude
        float distance = sqrt(pow(current_gps_->longitude - position_copy.longitude, 2) +
            pow(current_gps_->latitude - position_copy.latitude, 2)) * 111320.0; // Approx conversion from degrees to meters
        
        if(distance < 10.0) {
            double dx = (current_gps_->longitude - position_copy.longitude) * 111320.0;
            double dy = (current_gps_->latitude  - position_copy.latitude)  * 111320.0;
            float x_error =  cos(drone_yaw) * dx + sin(drone_yaw) * dy;
            float y_error = -sin(drone_yaw) * dx + cos(drone_yaw) * dy;
            RCLCPP_INFO(node_->get_logger(), "Close to goal, dx: %.2f m, dy: %.2f m", dx, dy);
            cmd_vel.linear.x = x_error * k_xy_/10;
            cmd_vel.linear.y = y_error * k_xy_/10;
            cmd_vel.angular.z = 0;
            if(position_copy.altitude > 10.0) {
                cmd_vel.linear.z = -0.5; // Descend if above 10m
            }
        }
        else if (twisting) {
            if (abs(bearing_error) < 0.05) {
                twisting = false; // Stop twisting when aligned
            }
            cmd_vel.angular.z = bearing_error * k_yaw_; // Yaw rate command
            cmd_vel.linear.x = 0.0; // No forward movement while twisting
        }
        else {
            if (abs(bearing_error) < 0.2) {
                cmd_vel.angular.z = bearing_error * k_yaw_;; // Yaw rate command
                cmd_vel.linear.x = distance * k_xy_; // Forward velocity command
            }
            else {
                cmd_vel.angular.z = bearing_error * k_yaw_; // Yaw rate command
                cmd_vel.linear.x = 0.0; // Forward velocity command
                twisting = true; // Re-enter twisting mode if misaligned
            }
        }
        std::cout << "Bearing: " << bearing << ", Drone Yaw: " << drone_yaw 
                  << ", Bearing Error: " << bearing_error << ", Distance: " << distance << std::endl;        
        if(distance < 0.5 || landing_init) { // Within 1 meter of goal or landing initiated
            RCLCPP_INFO(node_->get_logger(), "Reached GPS goal! or Landing initiated.");
            break;
        }
        else{
            cmd_vel_pub_->publish(cmd_vel);
        }
        
    }
    return BT::NodeStatus::SUCCESS;
}

void GoToGPS::positionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_position_ = msg;
}

void GoToGPS::attitudeCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_attitude_ = msg;
}
} //namespace bt_aruco_landing
