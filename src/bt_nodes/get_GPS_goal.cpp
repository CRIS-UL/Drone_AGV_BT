#include <bt_aruco_landing/bt_nodes/get_GPS_goal.hpp>

namespace bt_aruco_landing
{
GetGPSPose::GetGPSPose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{   
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    // Use SensorDataQoS for GPS data compatibility
    gps_goal_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/fixposition/gnss1", rclcpp::SensorDataQoS(), std::bind(&GetGPSPose::gpsCallback, this, std::placeholders::_1));
}
BT::NodeStatus GetGPSPose::tick()
{
    rclcpp::Time start_time = node_->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0.5);  // 0.5s wait
    while (rclcpp::ok() && (node_->now() - start_time) < timeout) {
        rclcpp::spin_some(node_);
        {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_gps_) {
            break;
        }
        }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_gps_) {
        setOutput("gps_goal", *current_gps_);
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_WARN(node_->get_logger(), "No GPS data received yet.");
        return BT::NodeStatus::FAILURE;
    }
}
void GetGPSPose::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_gps_ = msg;
}
} //namespace bt_aruco_landing
