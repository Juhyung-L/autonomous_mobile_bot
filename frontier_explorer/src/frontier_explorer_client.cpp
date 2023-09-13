#include "frontier_explorer/frontier_explorer_client.hpp"

namespace frontier_explorer
{
using namespace std::placeholders;

FrontierExplorerClient::FrontierExplorerClient(const rclcpp::Node::SharedPtr& node)
: node(node)
{
    action_client = rclcpp_action::create_client<ExploreFrontier>(
        node->get_node_base_interface(),
        node->get_node_graph_interface(),
        node->get_node_logging_interface(),
        node->get_node_waitables_interface(),
        "explore_frontier"
    );
}

void FrontierExplorerClient::sendGoal()
{
    RCLCPP_INFO(node->get_logger(), "Waiting for frontier explorer action server...");
    action_client->wait_for_action_server();
    RCLCPP_INFO(node->get_logger(), "frontier explorer action server available");

    auto goal = nav2_msgs::action::ExploreFrontier::Goal();

    RCLCPP_INFO(node->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ExploreFrontier>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&FrontierExplorerClient::goalResponseCallback, this, _1);
    send_goal_options.feedback_callback = std::bind(&FrontierExplorerClient::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&FrontierExplorerClient::resultCallback, this, _1);

    action_future = action_client->async_send_goal(goal, send_goal_options);
}

void FrontierExplorerClient::goalResponseCallback(const GoalHandleExploreFrontier::SharedPtr& goal_handle)
{
    if (!goal_handle) 
    {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    } 
    else 
    {
        RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void FrontierExplorerClient::feedbackCallback(
    GoalHandleExploreFrontier::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const nav2_msgs::action::ExploreFrontier::Feedback> feedback)
{
    RCLCPP_INFO(node->get_logger(), "Map size: %d", feedback->map_size);
}

void FrontierExplorerClient::resultCallback(const GoalHandleExploreFrontier::WrappedResult& result)
{
    switch (result.code) 
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(node->get_logger(), "Unknown result code");
            return;
    }
    action_result = result.result;
}

} // namespace frontier_explorer_client