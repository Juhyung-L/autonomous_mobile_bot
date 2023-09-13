#ifndef FRONTIER_EXPLORER__FRONTIER_EXPLORER_HPP_
#define FRONTIER_EXPLORER__FRONTIER_EXPLORER_HPP_

#include <chrono>
#include <string>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_msgs/action/explore_frontier.hpp"
#include "frontier_explorer/frontier_search.hpp"

namespace frontier_explorer
{
    using namespace std::placeholders; // NOLINT
    using namespace std::chrono_literals; // NOLINT

class FrontierExplorer : public nav2_util::LifecycleNode
{
public:
    using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    using ExploreFrontier = nav2_msgs::action::ExploreFrontier;
    using GoalHandleExploreFrontier = rclcpp_action::ServerGoalHandle<ExploreFrontier>;

    FrontierExplorer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

protected:
  /*
   * @brief Lifecycle configure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle activate
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle deactivate
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle cleanup
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle shutdown
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  
private:
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void planPath(const nav2_msgs::msg::Costmap::SharedPtr costmap);
    void reachedGoal(const NavigationGoalHandle::WrappedResult& result);
    bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose);
        
    std::string map_frame;
    std::string robot_frame;

    rclcpp::Logger logger = this->get_logger();

    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr move_base_client;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    std::shared_ptr<FrontierSearch> search;
    geometry_msgs::msg::Point prev_goal;
    double prev_distance_to_goal;
    rclcpp::Time last_progress;
    int progress_timeout;
    bool goal_received;
    double transform_tolerance;

    // action server stuff
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        const std::shared_ptr<const ExploreFrontier::Goal>& goal);
    
    void handle_accepted(const std::shared_ptr<GoalHandleExploreFrontier>& goal_handle);

    rclcpp_action::CancelResponse handle_canceled(
    const std::shared_ptr<GoalHandleExploreFrontier>& goal_handle);

    rclcpp_action::Server<ExploreFrontier>::SharedPtr action_server;
    std::shared_ptr<GoalHandleExploreFrontier> frontier_explorer_goal_handle;
    std::shared_future<NavigationGoalHandle::SharedPtr> move_base_future;
};

} // namespace frontier_explorer

#endif