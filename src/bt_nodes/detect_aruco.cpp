#include "bt_aruco_landing/bt_nodes/detect_aruco.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace bt_aruco_landing
{

using std::placeholders::_1;

DetectAruco::DetectAruco(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("detect_aruco_bt_node");
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/simple_drone/msdk_cmd_vel", rclcpp::QoS(10));
  pose_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/arucoPose", rclcpp::SensorDataQoS(),
    std::bind(&DetectAruco::poseCallback, this, _1));
  gimbal_client_ = node_->create_client<dji_msdk_ros::srv::GimbalAction>("/dji_msdk_ros/gimbal_action");

}

BT::PortsList DetectAruco::providedPorts()
{
  return {
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("aruco_pose"),
    BT::BidirectionalPort<bool>("landing_init")
  };
}

void DetectAruco::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_pose_ =  msg;
}

BT::NodeStatus DetectAruco::tick()
{
  rclcpp::Time start_time = node_->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(2.0);  // 2s wait
  auto request = std::make_shared<dji_msdk_ros::srv::GimbalAction::Request>();
  request->rotation_mode = 0;
  request->roll = 0.0;
  request->pitch = -90.0;
  request->yaw = 0.0;
  bool landing_init_;
  if (!getInput("landing_init", landing_init_)) {
    RCLCPP_WARN(node_->get_logger(), "No landing_init input, defaulting to false");
    landing_init_ = false;
  }
  
  // Send gimbal request asynchronously
  if (gimbal_client_->wait_for_service(std::chrono::seconds(1))) {
    auto future = gimbal_client_->async_send_request(request);
  } else {
    RCLCPP_WARN(node_->get_logger(), "Gimbal service not available");
  }
  while (rclcpp::ok() && (node_->now() - start_time) < timeout) {
    rclcpp::spin_some(node_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (latest_pose_) {
        break;
      }
    }
  }
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_pose_) {
      RCLCPP_WARN(node_->get_logger(), "Timeout waiting for pose message");
      if(!landing_init_){
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.linear.y = 0.0;
        stop_cmd.linear.z = 0.0;
        stop_cmd.angular.x = 0.0;
        stop_cmd.angular.y = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
      }
      return BT::NodeStatus::FAILURE;
    }
    pose= *latest_pose_;
  }
  RCLCPP_INFO(node_->get_logger(), "Aruco pose detected: %f, %f, %f",
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  setOutput("aruco_pose", pose);
  setOutput("landing_init", true);
  return BT::NodeStatus::SUCCESS;
}


}  // namespace bt_aruco_landing
