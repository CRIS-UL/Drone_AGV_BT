#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "bt_aruco_landing/bt_nodes/detect_aruco.hpp"
#include "bt_aruco_landing/bt_nodes/align_to_aruco.hpp"
#include "bt_aruco_landing/bt_nodes/land_on_marker.hpp"
#include "bt_aruco_landing/bt_nodes/go_to_GPS.hpp"
#include "bt_aruco_landing/bt_nodes/get_GPS_goal.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <optional>
#include <iostream>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto ros_node = rclcpp::Node::make_shared("bt_landing_node");

  // Store latest ArUco pose
  std::optional<geometry_msgs::msg::PoseStamped> latest_pose;
  std::mutex pose_mutex;

  // Subscribe to /aruco_pose and update the latest_pose
  auto sub = ros_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aruco_pose", 10,
    [&latest_pose, &pose_mutex](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(pose_mutex);
      latest_pose = *msg;
    });

  BT::BehaviorTreeFactory factory;

  // Create a shared blackboard and set the node
  auto global_blackboard = BT::Blackboard::create();
  global_blackboard->set<rclcpp::Node::SharedPtr>("node", ros_node);

  // Register custom BT nodes
  factory.registerNodeType<bt_aruco_landing::DetectAruco>("DetectAruco");
  factory.registerNodeType<bt_aruco_landing::AlignToAruco>("AlignToAruco");
  factory.registerNodeType<bt_aruco_landing::LandOnMarker>("LandOnMarker");
  factory.registerNodeType<bt_aruco_landing::GoToGPS>("GoToGPS");
  factory.registerNodeType<bt_aruco_landing::GetGPSPose>("GetGPSPose");
  
  std::string tree_file;
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("bt_aruco_landing");
  tree_file = pkg_share_dir + "/behaviour_trees/land_tree.xml";
  std::cout << "Loading behaviour tree from: " <<  tree_file << std::endl;

  // Create BT tree with the prepared blackboard
  auto tree = factory.createTreeFromFile(tree_file, global_blackboard);
  std::cout << "Created tree with root: " << std::endl;
  auto blackboard = tree.rootBlackboard();

  BT::StdCoutLogger logger(tree);
  BT::PublisherZMQ publisher_zmq(tree);
  RCLCPP_INFO(ros_node->get_logger(), "Behaviour tree loaded. Running...");

  BT::NodeStatus status = BT::NodeStatus::IDLE;

  while (rclcpp::ok())
  {
    // Update blackboard with latest pose if available
    {
      std::lock_guard<std::mutex> lock(pose_mutex);
      if (latest_pose.has_value())
      {
        blackboard->set("aruco_pose", latest_pose.value());
      }
    }

    status = tree.tickRoot();
    rclcpp::spin_some(ros_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(ros_node->get_logger(), "BT finished with status: %s", toStr(status, true).c_str());

  rclcpp::shutdown();
  return 0;
}
