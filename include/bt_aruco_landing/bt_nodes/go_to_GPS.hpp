#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <optional>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bt_aruco_landing
{
class GoToGPS : public BT::SyncActionNode
{
public:
    GoToGPS(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts(){
        return{ 
            BT::InputPort<rclcpp::Node::SharedPtr>("node"),
            BT::InputPort<sensor_msgs::msg::NavSatFix>("gps_goal"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("drone_pose")
        };
    }
    BT::NodeStatus tick() override;
private:
    rclcpp::Node::SharedPtr node_;
    sensor_msgs::msg::NavSatFix::SharedPtr current_gps_;
    geometry_msgs::msg::PoseStamped::SharedPtr drone_pose_;
    std::mutex mutex_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    float k_yaw_ = 0.1; // Proportional gain for yaw control
    float k_xy_ = 0.5;  // Proportional gain for horizontal velocity
};
} //namespace bt_aruco_landing