#ifndef FRONTIER_EXPLORER__FRONTIER_EXPLORER_CLIENT_HPP
#define FRONTIER_EXPLORER__FRONTIER_EXPLORER_CLIENT_HPP

#include <memory>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "mobile_bot_msgs/action/explore_frontier.hpp"

namespace frontier_explorer
{

class FrontierExplorerClient
{
public:
    using ExploreFrontier = mobile_bot_msgs::action::ExploreFrontier;
    using GoalHandleExploreFrontier = rclcpp_action::ClientGoalHandle<ExploreFrontier>;
    FrontierExplorerClient(const rclcpp::Node::SharedPtr& node);
    void sendGoal();
    std::shared_future<GoalHandleExploreFrontier::SharedPtr> action_status;
private:
    void goalResponseCallback(const GoalHandleExploreFrontier::SharedPtr& goal_handle);
    void feedbackCallback(
        GoalHandleExploreFrontier::SharedPtr goal_handle,
        const std::shared_ptr<const ExploreFrontier::Feedback> feedback);
    void resultCallback(const GoalHandleExploreFrontier::WrappedResult& result);

    rclcpp_action::Client<ExploreFrontier>::SharedPtr action_client;
    rclcpp::Node::SharedPtr node;
};

}
#endif