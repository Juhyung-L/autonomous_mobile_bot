#ifndef FRONTIER_EXPLORER__FRONTIER_EXPLORER_HPP_
#define FRONTIER_EXPLORER__FRONTIER_EXPLORER_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "frontier_explorer/frontier_search.hpp"

namespace frontier_explorer
{

using namespace std::placeholders; // NOLINT
using namespace std::chrono_literals; // NOLINT
using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>; // NOLINT

class FrontierExplorer : public rclcpp::Node
{
public:
    FrontierExplorer();
private:
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void planPath(const nav2_msgs::msg::Costmap::SharedPtr costmap);
    void reachedGoal(const NavigationGoalHandle::WrappedResult& result);
    geometry_msgs::msg::TransformStamped robotWorldPose();

    bool costmapReceived = false;
    bool initComplete = false;
    std::string mapFrame;
    std::string robotFrame;

    rclcpp::Logger logger = this->get_logger();

    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmapSub;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr moveBaseClient;

    rclcpp::TimerBase::SharedPtr pathPlanTimer;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    FrontierSearch search;
    geometry_msgs::msg::Point prevGoal;
    double prevDistanceToGoal;
    rclcpp::Time lastProgress;
    int progressTimeout;
};

} // namespace frontier_explorer

#endif