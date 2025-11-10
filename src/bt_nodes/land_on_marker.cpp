#include "bt_aruco_landing/bt_nodes/land_on_marker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/empty.hpp>
#include <chrono>
#include <thread>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace bt_aruco_landing
{

LandOnMarker::LandOnMarker(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("land_on_marker_bt_node");

  land_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
    "/simple_drone/land", rclcpp::QoS(10));
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/simple_drone/msdk_cmd_vel", rclcpp::QoS(10));
}

BT::PortsList LandOnMarker::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("aruco_pose")
  };
}
BT::NodeStatus LandOnMarker::tick()
{
  geometry_msgs::msg::PoseStamped marker_pose;
  if (!getInput("aruco_pose", marker_pose)) {
    RCLCPP_WARN(node_->get_logger(), "No ArUco pose input");
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(node_->get_logger(), "Landing initiated");
  rclcpp::Time start_time = node_->now();
  if (marker_pose.pose.position.z < 0.5) {
    std_msgs::msg::Empty msg;
    land_pub_->publish(msg);

    // Give the drone some time to respond before succeeding
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    config().blackboard->set("land_requested", false);
    return BT::NodeStatus::SUCCESS;
  } else {
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = -0.5;  // Move downwards
    cmd_vel_pub_->publish(cmd_vel_msg);

    // Give the drone some time to respond before checking again
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return BT::NodeStatus::FAILURE;

  }
}

}  // namespace bt_aruco_landing