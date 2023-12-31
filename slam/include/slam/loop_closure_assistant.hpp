/*
 * loop_closure_assistant
 * Copyright (c) 2019, Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#ifndef SLAM__LOOP_CLOSURE_ASSISTANT_HPP_
#define SLAM__LOOP_CLOSURE_ASSISTANT_HPP_

#include <thread>
#include <string>
#include <functional>
#include <memory>
#include <map>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"

#include "slam/toolbox_types.hpp"
#include "slam/laser_utils.hpp"
#include "slam/visualization_utils.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace loop_closure_assistant
{

using namespace ::toolbox_types;  // NOLINT

class LoopClosureAssistant
{
public:
  LoopClosureAssistant(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, karto::Mapper * mapper,
    laser_utils::ScanHolder * scan_holder, PausedState & state,
    ProcessType & processor_type);

  void clearMovedNodes();
  void processInteractiveFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);
  void publishGraph();
  void setMapper(karto::Mapper * mapper);

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;

private:
  bool manualLoopClosureCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam::srv::LoopClosure::Request> req, 
    std::shared_ptr<slam::srv::LoopClosure::Response> resp);
  bool clearChangesCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam::srv::Clear::Request> req, 
    std::shared_ptr<slam::srv::Clear::Response> resp);
  bool interactiveModeCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<slam::srv::ToggleInteractive::Request>  req,
    std::shared_ptr<slam::srv::ToggleInteractive::Response> resp);

  void moveNode(const int& id, const Eigen::Vector3d& pose);
  void addMovedNodes(const int& id, Eigen::Vector3d vec);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
  laser_utils::ScanHolder * scan_holder_;
  rclcpp::Service<slam::srv::Clear>::SharedPtr ssClear_manual_;
  rclcpp::Service<slam::srv::LoopClosure>::SharedPtr ssLoopClosure_;
  rclcpp::Service<slam::srv::ToggleInteractive>::SharedPtr ssInteractive_;
  boost::mutex moved_nodes_mutex_;
  std::map<int, Eigen::Vector3d> moved_nodes_;
  karto::Mapper * mapper_;
  karto::ScanSolver * solver_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;
  boost::mutex interactive_mutex_;
  bool interactive_mode_, enable_interactive_mode_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string map_frame_;
  PausedState & state_;
  ProcessType & processor_type_;
};

}   // namespace loop_closure_assistant

#endif  // slam__LOOP_CLOSURE_ASSISTANT_HPP_
