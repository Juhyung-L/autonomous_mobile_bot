#ifndef FRONTIER_EXPLORER__FRONTIER_EXPLORER_CLIENT_HPP
#define FRONTIER_EXPLORER__FRONTIER_EXPLORER_CLIENT_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"

#include "mobile_bot_msgs/action/explore_frontier.hpp"

namespace frontier_explorer_client
{

class FrontierExplorerClient : public rclcpp::Node
{
public:
    using ExploreFrontier = mobile_bot_msgs::action::ExploreFrontier;
    using GoalHandleExploreFrontier = rclcpp_action::ClientGoalHandle<ExploreFrontier>;
    FrontierExplorerClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
private:
    void sendGoal(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void goalResponseCallback(const GoalHandleExploreFrontier::SharedPtr& goal_handle);
    void feedbackCallback(
        GoalHandleExploreFrontier::SharedPtr goal_handle,
        const std::shared_ptr<const ExploreFrontier::Feedback> feedback);
    void resultCallback(const GoalHandleExploreFrontier::WrappedResult& result);

    rclcpp_action::Client<ExploreFrontier>::SharedPtr action_client;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_server;
};

}
#endif