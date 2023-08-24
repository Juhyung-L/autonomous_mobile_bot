#include "frontier_explorer/frontier_explorer_client.hpp"

namespace frontier_explorer_client
{
using namespace std::placeholders;

FrontierExplorerClient::FrontierExplorerClient(const rclcpp::NodeOptions& options)
: Node("frontier_explorer_client", "", options)
{
    action_client = rclcpp_action::create_client<ExploreFrontier>(
        this,
        "explore_frontier"
    );
    service_server = create_service<std_srvs::srv::Empty>(
        "explore_frontier_send_goal",
        std::bind(&FrontierExplorerClient::sendGoal, this, _1, _2)
    );
}

void FrontierExplorerClient::sendGoal(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
    RCLCPP_INFO(get_logger(), "Waiting for frontier explorer action server...");
    action_client->wait_for_action_server();
    RCLCPP_INFO(get_logger(), "frontier explorer action server available");

    auto goal = ExploreFrontier::Goal();

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<ExploreFrontier>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&FrontierExplorerClient::goalResponseCallback, this, _1);
    send_goal_options.feedback_callback = std::bind(&FrontierExplorerClient::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&FrontierExplorerClient::resultCallback, this, _1);

    action_client->async_send_goal(goal, send_goal_options);
}

void FrontierExplorerClient::goalResponseCallback(const GoalHandleExploreFrontier::SharedPtr& goal_handle)
{
    if (!goal_handle) 
    {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    } 
    else 
    {
        RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    }
}

void FrontierExplorerClient::feedbackCallback(
    GoalHandleExploreFrontier::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const ExploreFrontier::Feedback> feedback)
{
    RCLCPP_INFO(get_logger(), "Map size: %d", feedback->map_size);
}

void FrontierExplorerClient::resultCallback(const GoalHandleExploreFrontier::WrappedResult& result)
{
    switch (result.code) 
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            return;
    }

    if (result.result->all_frontiers_cleared)
    {
        RCLCPP_INFO(get_logger(), "All frontiers cleared!");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Not all frontiers cleared.");
    }
}

} // namespace frontier_explorer_client