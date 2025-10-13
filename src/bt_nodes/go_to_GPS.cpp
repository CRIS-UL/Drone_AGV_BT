#include "bt_aruco_landing/bt_nodes/go_to_GPS.hpp"

namespace bt_aruco_landing
{
GoToGPS::GoToGPS(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config){
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/dji_msdk_ros/cmd_vel", 10);
    }
BT::NodeStatus GoToGPS::tick()
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto gps_input = this->getInput<sensor_msgs::msg::NavSatFix>("gps_goal");
    auto pose_input = this->getInput<geometry_msgs::msg::PoseStamped>("drone_pose");
    
    if (!gps_input || !pose_input) {
        RCLCPP_WARN(node_->get_logger(), "Missing GPS goal or drone pose input.");
        return BT::NodeStatus::FAILURE;
    }
    
    current_gps_ = std::make_shared<sensor_msgs::msg::NavSatFix>(gps_input.value());
    drone_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(pose_input.value());
    // Get bearing in NED frame
    float bearing = atan2(current_gps_->longitude - drone_pose_->pose.position.x,
                        current_gps_->latitude - drone_pose_->pose.position.y);
    float drone_yaw = tf2::getYaw(drone_pose_->pose.orientation);
    float bearing_error = bearing - drone_yaw;
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.angular.z = bearing_error * k_yaw_; // Yaw rate command
    float distance = sqrt(pow(current_gps_->longitude - drone_pose_->pose.position.x, 2) +
                          pow(current_gps_->latitude - drone_pose_->pose.position.y, 2));
    if (bearing_error < 0.1 && bearing_error > -0.1) {
        cmd_vel.linear.x = distance * k_xy_; // Forward velocity command
    } else {
        cmd_vel.linear.x = 0.0; // Stop forward movement while turning
    }
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0; // Maintain current altitude
    cmd_vel_pub_->publish(cmd_vel);
    return BT::NodeStatus::SUCCESS;
};
} //namespace bt_aruco_landing