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

  pose_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/arucoPose", rclcpp::SensorDataQoS(),
    std::bind(&DetectAruco::poseCallback, this, _1));
}

BT::PortsList DetectAruco::providedPorts()
{
  return {
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("aruco_pose")
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
      return BT::NodeStatus::FAILURE;
    }
    pose= *latest_pose_;
  }
  RCLCPP_INFO(node_->get_logger(), "Aruco pose detected: %f, %f, %f",
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  setOutput("aruco_pose", pose);
  return BT::NodeStatus::SUCCESS;
}


}  // namespace bt_aruco_landing
