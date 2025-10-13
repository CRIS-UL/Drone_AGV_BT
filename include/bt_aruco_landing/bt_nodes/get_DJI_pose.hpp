#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <optional>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace bt_aruco_landing
{
class GetDJIPose : public BT::SyncActionNode
{
public:
    GetDJIPose(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts(){
        return{ BT::OutputPort<geometry_msgs::msg::PoseStamped>("dji_pose")
            };
    };
    BT::NodeStatus tick() override;
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_sub_;
    sensor_msgs::msg::NavSatFix::SharedPtr current_position_;
    geometry_msgs::msg::Vector3::SharedPtr current_attitude_;

    std::mutex mutex_;
    void positionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void attitudeCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
};
} //namespace bt_aruco_landing
