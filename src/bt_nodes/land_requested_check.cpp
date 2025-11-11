#include "bt_aruco_landing/bt_nodes/land_requested_check.hpp"
namespace bt_aruco_landing
{
LandRequestedCheck::LandRequestedCheck(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config){
        prev_land_requested_ = false;
        enable_virtual_sticks_pub_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node")->create_publisher<std_msgs::msg::Empty>(
            "/simple_drone/enable_virtual_sticks", 10);
}
BT::NodeStatus LandRequestedCheck::tick()
{
    // Access the blackboard to check if landing is requested
    auto blackboard = config().blackboard;
    bool land_requested = blackboard->get<bool>("land_requested");
    
    if (land_requested) {
        if(prev_land_requested_){
            std_msgs::msg::Empty msg;
            enable_virtual_sticks_pub_->publish(msg);
        }
        prev_land_requested_ = true;

        return BT::NodeStatus::SUCCESS;
    } else {
        prev_land_requested_ = false;
        return BT::NodeStatus::FAILURE;
    }
}
} //namespace bt_aruco_landing