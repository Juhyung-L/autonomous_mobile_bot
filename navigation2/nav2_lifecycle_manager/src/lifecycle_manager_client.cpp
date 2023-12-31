// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"

namespace nav2_lifecycle_manager
{
LifecycleManagerClient::LifecycleManagerClient(
  const std::string & name,
  std::shared_ptr<rclcpp::Node> parent_node)
{
  manage_service_name_ = name + std::string("/manage_nodes");
  active_service_name_ = name + std::string("/is_active");
  add_node_service_name_ = name + std::string("/add_node");
  remove_node_service_name_ = name + std::string("/remove_node");

  // Use parent node for service call and logging
  node_ = parent_node;

  // Create the service clients
  manager_client_ = std::make_shared<nav2_util::ServiceClient<ManageLifecycleNodes>>(
    manage_service_name_, node_);
  is_active_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::Trigger>>(
    active_service_name_, node_);
  add_node_client_ = std::make_shared<nav2_util::ServiceClient<nav2_msgs::srv::AddNode>>(
    add_node_service_name_, node_);
  remove_node_client_ = std::make_shared<nav2_util::ServiceClient<nav2_msgs::srv::RemoveNode>>(
    remove_node_service_name_, node_);
}

bool
LifecycleManagerClient::startup(const std::chrono::nanoseconds timeout)
{
  return callService(ManageLifecycleNodes::Request::STARTUP, timeout);
}

bool
LifecycleManagerClient::shutdown(const std::chrono::nanoseconds timeout)
{
  return callService(ManageLifecycleNodes::Request::SHUTDOWN, timeout);
}

bool
LifecycleManagerClient::pause(const std::chrono::nanoseconds timeout)
{
  return callService(ManageLifecycleNodes::Request::PAUSE, timeout);
}

bool
LifecycleManagerClient::resume(const std::chrono::nanoseconds timeout)
{
  return callService(ManageLifecycleNodes::Request::RESUME, timeout);
}

bool
LifecycleManagerClient::reset(const std::chrono::nanoseconds timeout)
{
  return callService(ManageLifecycleNodes::Request::RESET, timeout);
}

SystemStatus
LifecycleManagerClient::is_active(const std::chrono::nanoseconds timeout)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();

  RCLCPP_DEBUG(
    node_->get_logger(), "Waiting for the %s service...",
    active_service_name_.c_str());

  if (!is_active_client_->wait_for_service(timeout)) {
    return SystemStatus::TIMEOUT;
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "Sending %s request",
    active_service_name_.c_str());

  try {
    response = is_active_client_->invoke(request, timeout);
  } catch (std::runtime_error &) {
    return SystemStatus::TIMEOUT;
  }

  if (response->success) {
    return SystemStatus::ACTIVE;
  } else {
    return SystemStatus::INACTIVE;
  }
}

bool
LifecycleManagerClient::add_node(std::vector<std::string> node_names, const std::chrono::nanoseconds timeout)
{
  auto request = std::make_shared<nav2_msgs::srv::AddNode::Request>();
  auto response = std::make_shared<nav2_msgs::srv::AddNode::Response>();
  request->node_names = node_names;

  RCLCPP_DEBUG(
    node_->get_logger(), "Waiting for the %s service...",
    add_node_service_name_.c_str());

  if (!add_node_client_->wait_for_service(timeout)) {
    return false;
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "Sending %s request",
    add_node_service_name_.c_str());
  
  try {
    response = add_node_client_->invoke(request, timeout);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(node_->get_logger(), "Add node service error: %s", e.what());
    return false;
  }
  return response->success;
}

bool 
LifecycleManagerClient::remove_node(std::vector<std::string> node_names, const std::chrono::nanoseconds timeout)
{
  auto request = std::make_shared<nav2_msgs::srv::RemoveNode::Request>();
  auto response = std::make_shared<nav2_msgs::srv::RemoveNode::Response>();
  request->node_names = node_names;

  RCLCPP_DEBUG(
    node_->get_logger(), "Waiting for the %s service...",
    remove_node_service_name_.c_str());

  if (!remove_node_client_->wait_for_service(timeout)) {
    return false;
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "Sending %s request",
    remove_node_service_name_.c_str());

  // if sending request to service server, wrap the function with try-catch block
  try {
    response = remove_node_client_->invoke(request, timeout);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(node_->get_logger(), "Remove node service error: %s", e.what());
    return false;
  }
  return response->success;
}

bool
LifecycleManagerClient::callService(uint8_t command, const std::chrono::nanoseconds timeout)
{
  auto request = std::make_shared<ManageLifecycleNodes::Request>();
  auto response = std::make_shared<ManageLifecycleNodes::Response>();
  request->command = command;

  RCLCPP_DEBUG(
    node_->get_logger(), "Waiting for the %s service...",
    manage_service_name_.c_str());

  if (!manager_client_->wait_for_service(timeout)) {
    return false;
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "Sending %s request",
    manage_service_name_.c_str());
  try {
    response = manager_client_->invoke(request, timeout);
    return response->success;
  } catch (std::runtime_error &) {
    return false;
  }
}

}  // namespace nav2_lifecycle_manager
