#include <bt_aruco_landing/bt_nodes/get_DJI_pose.hpp>

namespace bt_aruco_landing
{
GetDJIPose::GetDJIPose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config){
    node_ = rclcpp::Node::make_shared("node");
    attitude_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        this->getInput<std::string>("/dji_msdk_ros/drone_attitude").value(),
        10,
        std::bind(&GetDJIPose::attitudeCallback, this, std::placeholders::_1));
    position_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        this->getInput<std::string>("/dji_msdk_ros/gps_position").value(),
        10,
        std::bind(&GetDJIPose::positionCallback, this, std::placeholders::_1));
}
BT::NodeStatus GetDJIPose::tick()
{
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->now();
    pose.header.frame_id = "gps_frame";
    if (current_attitude_ && current_position_) {
        pose.pose.position.x = current_position_->longitude;
        pose.pose.position.y = current_position_->latitude;
        pose.pose.position.z = current_position_->altitude;
        // Convert Euler angles (roll, pitch, yaw) to quaternion
        tf2::Quaternion q;
        q.setRPY(current_attitude_->x, current_attitude_->y, current_attitude_->z);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_WARN(node_->get_logger(), "No DJI data received yet.");
        return BT::NodeStatus::FAILURE;
    }
}
void GetDJIPose::positionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_position_ = msg;
}
void GetDJIPose::attitudeCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_attitude_ = msg;
}
} //namespace bt_aruco_landing