#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
namespace bt_aruco_landing
{
class LandRequestedCheck : public BT::SyncActionNode
{
public:
    LandRequestedCheck(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts(){
        return{};
    }  
private:
    bool prev_land_requested_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr enable_virtual_sticks_pub_;
};
} //namespace bt_aruco_landing