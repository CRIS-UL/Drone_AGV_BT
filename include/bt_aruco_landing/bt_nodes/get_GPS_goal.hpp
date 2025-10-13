#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <optional>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
namespace bt_aruco_landing
{
class GetGPSPose : public BT::SyncActionNode
{
public:
    GetGPSPose(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts(){
        return{ 
            BT::InputPort<rclcpp::Node::SharedPtr>("node"),
            BT::OutputPort<sensor_msgs::msg::NavSatFix>("gps_goal")
        };
    }
    BT::NodeStatus tick() override;
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_goal_;
    sensor_msgs::msg::NavSatFix::SharedPtr current_gps_;

    std::mutex mutex_;
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};
} //namespace bt_aruco_landing
