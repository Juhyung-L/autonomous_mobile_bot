#include <vector>
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "frontier_explorer/frontier_explorer_server.hpp"

namespace frontier_explorer
{

static bool samePoint(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.1;
}

FrontierExplorer::FrontierExplorer(const rclcpp::NodeOptions& options)
: nav2_util::LifecycleNode("frontier_explorer_server", "", options)
{ 
    this->declare_parameter<std::string>("robot_frame", "base_footprint");
    this->declare_parameter<int>("progress_timeout", 10);

    // robot's base footprint
    this->get_parameter("robot_frame", robot_frame);
    // time required for a goal to be considered unreachable and be put into the blacklist
    this->get_parameter("progress_timeout", progress_timeout);
}

nav2_util::CallbackReturn FrontierExplorer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    goal_received = false;

    tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    search = std::make_shared<FrontierSearch>(shared_from_this());

    costmap_sub = this->create_subscription<nav2_msgs::msg::Costmap>(
        "global_costmap/costmap_raw", 10, std::bind(&FrontierExplorer::costmapCallback, this, _1));
    
    move_base_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    // block until move base server is online
    RCLCPP_INFO(logger, "Waiting for move base action server...");
    move_base_client->wait_for_action_server();
    RCLCPP_INFO(logger, "Move base action server is online!");

    action_server = rclcpp_action::create_server<ExploreFrontier>(
        this,
        "explore_frontier",
        std::bind(&FrontierExplorer::handle_goal, this, _1, _2),
        std::bind(&FrontierExplorer::handle_canceled, this, _1),
        std::bind(&FrontierExplorer::handle_accepted, this, _1)
    );

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn FrontierExplorer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger, "Activating frontier_explorer action server");
    search->marker_pub->on_activate();
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn FrontierExplorer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger, "Deactivating frontier_explorer action server");
    search->marker_pub->on_deactivate();

    RCLCPP_INFO(logger, "Canceling all move base actions");
    if (move_base_future.get()->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
    {
        move_base_client->async_cancel_all_goals();
    }

    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn FrontierExplorer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger, "Cleaning up frontier_explorer action server");

    // what if on_cleanup() is called when planPath() hasn't finished running
    // then it will result in invalid memory access because all of the resources are freed here
    frontier_explorer_goal_handle.reset();
    action_server.reset();
    costmap_sub.reset();
    move_base_client.reset();
    tf_buffer.reset();
    tf_listener.reset();
    search.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn FrontierExplorer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger, "Shutting down frontier_explorer action server");
    return nav2_util::CallbackReturn::SUCCESS;
}

void FrontierExplorer::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{   
    map_frame = msg->header.frame_id;
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE
        && goal_received)
    {
        planPath(msg);
    } 
}

void FrontierExplorer::planPath(const nav2_msgs::msg::Costmap::SharedPtr costmap)
{
    // get robot pose
    // then get the nearest free cell
    std::vector<Frontiers> frontiers_list;
    search->updateMap(costmap);

    // try to get map -> robot transform
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer->lookupTransform(map_frame, robot_frame, this->now(), 100ms);
    }
    catch (const std::runtime_error& e)
    {
        RCLCPP_WARN(logger, "Could not get map -> robot transform: %s", e.what());
        return;
    }
    
    geometry_msgs::msg::Pose robot_pose;
    robot_pose.position.x = transform.transform.translation.x;
    robot_pose.position.y = transform.transform.translation.y;
    frontiers_list = search->searchFrontiers(robot_pose);

    // publish feedback to action client
    auto feedback = std::make_shared<ExploreFrontier::Feedback>();
    feedback->map_size = costmap->metadata.size_x * costmap->metadata.size_y;
    frontier_explorer_goal_handle->publish_feedback(feedback);

    // sort the frontiers
    // pick the frontiers with the lowest cost that is not in the black list

    // if the picked goal is the same as the previous goal
    // check if the robot has made progress towards the goal
    // yes: continue no: put the goal in the black list

    // if the picked goal is new, send the move_base action

    // sort frontiers_list based on cost
    std::sort(frontiers_list.begin(), frontiers_list.end(),
                [](const Frontiers& f1, const Frontiers& f2) { return f1.cost < f2.cost;});
    search->visualizeFrontiers(frontiers_list);

    // picked the frontiers with the lowest cost that is not on the blacklist
    auto goal_frontiers = 
    std::find_if_not(frontiers_list.begin(), frontiers_list.end(),
                        [this](const Frontiers& f) {return search->goalOnBlacklist(f.centriod);
                        });

    if (goal_frontiers == frontiers_list.end())
    {
        RCLCPP_INFO(logger, "No frontiers left to explore, sending result to ExploreFrontier action client");
        auto result = std::make_shared<ExploreFrontier::Result>();
        if (frontiers_list.empty())
        {
            result->all_frontiers_cleared = true;
        }
        else 
        {
            result->all_frontiers_cleared = false;
        }
        frontier_explorer_goal_handle->succeed(result); // send result of action
        RCLCPP_INFO(logger, "Canceling all move base actions"); // cancel move base action
        if (move_base_future.get()->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
        {
            move_base_client->async_cancel_all_goals();
        }
        goal_received = false;
        return;
    }

    geometry_msgs::msg::Point goal = goal_frontiers->centriod;

    bool same_goal = samePoint(goal, prev_goal);
    prev_goal = goal;

    if (!same_goal || prev_distance_to_goal > goal_frontiers->min_distance)
    {
        // if not the same goal
        // or same goal but made some progress
        last_progress = this->now();
        prev_distance_to_goal = goal_frontiers->min_distance;
    }

    if (this->now() - last_progress > tf2::durationFromSec(progress_timeout))
    {
        search->addToBlacklist(goal);
        return;
    }

    if (same_goal)
    {
        return; // if same goal, don't need to send goal action
    }
    // at if the code gets here, it means the frontier is new and not on the blacklist

    RCLCPP_INFO(logger, "Sending goal to move_base server");
    auto move_base_goal = nav2_msgs::action::NavigateToPose::Goal();
    move_base_goal.pose.pose.position = goal;
    move_base_goal.pose.pose.orientation.w = 1.;
    move_base_goal.pose.header.frame_id = costmap->header.frame_id;
    move_base_goal.pose.header.stamp = this->now();

    // set result callback
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = 
    [this](const NavigationGoalHandle::WrappedResult& result) {reachedGoal(result);};

    // send goal action command
    move_base_future = move_base_client->async_send_goal(move_base_goal, send_goal_options);
}

void FrontierExplorer::reachedGoal(const NavigationGoalHandle::WrappedResult& result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_DEBUG(logger, "Goal was successful");
        break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_DEBUG(logger, "Goal was aborted");
            // search.addToBlacklist(sentGoal);
            // If it was aborted probably because we've found another frontier goal,
            // so just return and don't make plan again
        break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_DEBUG(logger, "Goal was canceled");
            // If goal canceled might be because exploration stopped from topic. Don't make new plan.
        break;
        default:
            RCLCPP_WARN(logger, "Unknown result code from move base nav2");
        break;
    }
}

// action server callbacks
rclcpp_action::GoalResponse FrontierExplorer::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    const std::shared_ptr<const ExploreFrontier::Goal>& /*goal*/)
{
    RCLCPP_INFO(logger, "Explore frontier goal received");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void FrontierExplorer::handle_accepted(
    const std::shared_ptr<GoalHandleExploreFrontier>& goal_handle)
{
    RCLCPP_INFO(logger, "Explore frontier goal accepted");
    goal_received = true;
    frontier_explorer_goal_handle = goal_handle;
}

rclcpp_action::CancelResponse FrontierExplorer::handle_canceled(
    const std::shared_ptr<GoalHandleExploreFrontier>& /*goal_handle*/)
{
    RCLCPP_INFO(logger, "Explore frontier goal canceled");
    RCLCPP_INFO(logger, "Canceling all move base actions"); // cancel move base action
    if (move_base_future.get()->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
    {
        move_base_client->async_cancel_all_goals();
    }
    goal_received = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

} // namespace frontier_explorer
