#include "bt_aruco_landing/bt_nodes/land_requested_check.hpp"
namespace bt_aruco_landing
{
LandRequestedCheck::LandRequestedCheck(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config){
}
BT::NodeStatus LandRequestedCheck::tick()
{
    // Access the blackboard to check if landing is requested
    auto blackboard = config().blackboard;
    bool land_requested = blackboard->get<bool>("land_requested");
    
    if (land_requested) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}
} //namespace bt_aruco_landing