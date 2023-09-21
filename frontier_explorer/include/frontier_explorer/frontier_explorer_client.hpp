#ifndef FRONTIER_EXPLORER__FRONTIER_EXPLORER_CLIENT_HPP
#define FRONTIER_EXPLORER__FRONTIER_EXPLORER_CLIENT_HPP

#include <memory>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_msgs/action/explore_frontier.hpp"

namespace frontier_explorer
{
using ExploreFrontier = nav2_msgs::action::ExploreFrontier;
using GoalHandleExploreFrontier = rclcpp_action::ClientGoalHandle<ExploreFrontier>;

class FrontierExplorerClient
{
public:
    FrontierExplorerClient(const rclcpp::Node::SharedPtr& node);
    void sendGoal();
    std::shared_future<GoalHandleExploreFrontier::SharedPtr> getGoalHandleFuture();
    bool getEndPose(geometry_msgs::msg::PoseStamped& input_pose);
    
private:
    void goalResponseCallback(const GoalHandleExploreFrontier::SharedPtr& goal_handle);
    void feedbackCallback(
        GoalHandleExploreFrontier::SharedPtr goal_handle,
        const std::shared_ptr<const ExploreFrontier::Feedback> feedback);
    void resultCallback(const GoalHandleExploreFrontier::WrappedResult& result);

    rclcpp_action::Client<ExploreFrontier>::SharedPtr action_client_;
    rclcpp::Node::SharedPtr node_;
    std::shared_future<GoalHandleExploreFrontier::SharedPtr> goal_handle_future_;
    geometry_msgs::msg::PoseStamped end_pose_;
};

}
#endif