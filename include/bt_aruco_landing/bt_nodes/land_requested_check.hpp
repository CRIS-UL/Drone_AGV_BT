#pragma once

#include <behaviortree_cpp_v3/action_node.h>
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
};
} //namespace bt_aruco_landing