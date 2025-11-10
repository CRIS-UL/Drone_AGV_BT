#include "bt_aruco_landing/bt_nodes/align_to_aruco.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <cmath>

namespace bt_aruco_landing
{

AlignToAruco::AlignToAruco(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)  // FIXED HERE
{
  node_ = rclcpp::Node::make_shared("align_to_aruco_bt_node");
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/simple_drone/msdk_cmd_vel", rclcpp::QoS(10));
}

BT::PortsList AlignToAruco::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("aruco_pose")
  };
}

BT::NodeStatus AlignToAruco::tick()
{
  rclcpp::spin_some(node_);

  geometry_msgs::msg::PoseStamped marker_pose;
  if (!getInput("aruco_pose", marker_pose)) {
    RCLCPP_WARN(node_->get_logger(), "No ArUco pose input");
    return BT::NodeStatus::FAILURE;
  }

  float dx = marker_pose.pose.position.x;  // Assuming the marker is aligned with the y-axis
  float dy = marker_pose.pose.position.y; // Assuming the marker is aligned with the x-axis
  tf2::Quaternion q(
    marker_pose.pose.orientation.x,
    marker_pose.pose.orientation.y,
    marker_pose.pose.orientation.z,
    marker_pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // Desired yaw is 0 (aligned), so error is just -yaw
  float yaw_error = yaw;

    // PID parameters for yaw
  float yaw_tolerance = 0.5; // ~10 degrees
  const float kp_yaw = 1.5;
  const float ki_yaw = 0.1;
  const float kd_yaw = 0.5;
  // PID parameters for spatial alignment
  float tolerance = 0.2;
  const float kp = 0.3;
  const float ki = 0.01;
  const float kd = 0.05;

  geometry_msgs::msg::Twist cmd_vel;
  rclcpp::Time now = node_->now();
  double dt = 0.05; // default to 50ms if first run
  if (prev_time_.nanoseconds() > 0) {
    dt = (now - prev_time_).seconds();
  }
  prev_time_ = now;

  // Integral
  integral_dx_ += dx * dt;
  integral_dy_ += dy * dt;
  integral_yaw_error_ += yaw_error * dt;
  // Derivative
  float derivative_dx = (dt > 0) ? (dx - prev_dx_) / dt : 0.0;
  float derivative_dy = (dt > 0) ? (dy - prev_dy_) / dt : 0.0;
  float derivative_yaw = (dt > 0) ? (yaw - prev_yaw_error_) / dt : 0.0;
  bool aligned = true;
  if(marker_pose.pose.position.z>30){
    tolerance=0.8;
    yaw_tolerance=1;
  } else if(marker_pose.pose.position.z>3){
    tolerance=0.05;
    yaw_tolerance=0.5;
  }
  if (std::abs(dx) > tolerance) {
    cmd_vel.linear.y = -(kp * dx + ki * integral_dx_ + kd * derivative_dx);
    aligned = false;
  } else {
    integral_dx_ = 0.0; // reset integral when within tolerance
  }

  if (std::abs(dy) > tolerance) {
    cmd_vel.linear.x = -(kp * dy + ki * integral_dy_ + kd * derivative_dy);
    aligned = false;
  } else {
    integral_dy_ = 0.0;
  }

  prev_dx_ = dx;
  prev_dy_ = dy;
  prev_yaw_error_  = yaw_error;

  if (aligned) {
    RCLCPP_INFO(node_->get_logger(), "Spatial Alignment complete");
    if (std::abs(yaw_error) > yaw_tolerance) {
      cmd_vel.angular.z = kp_yaw * yaw_error + ki_yaw * integral_yaw_error_ + kd_yaw * derivative_yaw;
      RCLCPP_INFO(node_->get_logger(), "Yaw error: %.2f, cmd_vel.angular.z: %.2f", yaw_error, cmd_vel.angular.z);
      cmd_vel_pub_->publish(cmd_vel);
      return BT::NodeStatus::FAILURE;
    }
    else {
      RCLCPP_INFO(node_->get_logger(), "Yaw aligned: %.2f", yaw_error);
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "Not aligned yet: dx=%.2f, dy=%.2f", dx, dy);
    cmd_vel_pub_->publish(cmd_vel);
    RCLCPP_INFO(node_->get_logger(), "Publishing cmd_vel: linear.x=%.2f, linear.y=%.2f",
                 cmd_vel.linear.x, cmd_vel.linear.y);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace bt_aruco_landing
