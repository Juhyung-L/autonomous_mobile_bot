#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "frontier_explorer/frontier_search.h"
#include "mobile_bot_msgs/action/autonomous_slam.hpp"

// action server
// handle_goal
// handle_cancel
// handel_accepted
// execute

// client
// goal response callback
// feedback callback
// result callback

using namespace std::placeholders;
using namespace std::chrono_literals;

static bool samePoint(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.1;
}

class AutonomousSlamActionServer : public rclcpp::Node
{
    using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    using AutoSlam = mobile_bot_msgs::action::AutonomousSlam;
    using GoalHandleAutoSlam = rclcpp_action::ServerGoalHandle<AutoSlam>;
public:
    AutonomousSlamActionServer()
    : Node("autonomous_slam_action_server")
    , tfBuffer(this->get_clock())
    , tfListener(tfBuffer)
    , search(*this) 
    {
        this->declare_parameter<std::string>("robot_frame", "base_footprint");
        this->declare_parameter<int>("progress_timeout", 10);

        this->get_parameter("robot_frame", robotFrame);
        this->get_parameter("progress_timeout", progressTimeout);
        // time required for a goal to be considered unreachable and be put into the blacklist

        action_server = rclcpp_action::create_server<AutoSlam>(
            this,
            "autonomous_slam",
            std::bind(&AutonomousSlamActionServer::handle_goal, this, _1, _2),
            std::bind(&AutonomousSlamActionServer::handle_cancel, this, _1),
            std::bind(&AutonomousSlamActionServer::handle_accepted, this, _1)
        );

        costmapSub = this->create_subscription<nav2_msgs::msg::Costmap>(
            "global_costmap/costmap_raw", 10, std::bind(&AutonomousSlamActionServer::costmapCallback, this, _1));

        moveBaseClient = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        // block until move base server is online
        RCLCPP_INFO(logger, "Waiting for move base action server...");
        moveBaseClient->wait_for_action_server();
        RCLCPP_INFO(logger, "Move base action server is online!");

        // block until receiving the first costmap message
        RCLCPP_INFO(logger, "Waiting for costmap...");
        while (rclcpp::ok() && !costmapReceived)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(10000000ns); // 100 Hz
        }
        RCLCPP_INFO(logger, "Costmap received!");

        // block until the map -> robot transform is available
        RCLCPP_INFO(logger, "Waiting for %s -> %s transform to become available", 
                    mapFrame.c_str(), robotFrame.c_str());
        auto lastError = this->now();
        std::string tfError;
        while (rclcpp::ok() && !tfBuffer.canTransform(mapFrame, robotFrame, 
               tf2::TimePointZero, 100ms, &tfError))
        {
            rclcpp::spin_some(this->get_node_base_interface());
            // don't need to sleep because canTransform() blocks for 100ms (100Hz)
            // print tf error message every 5 seconds
            if (lastError + 5s < this->now())
            {
                RCLCPP_WARN(logger, "Time out waiting for transform %s -> %s due to %s",
                            mapFrame.c_str(), robotFrame.c_str(), tfError.c_str());
                lastError = this->now();
            }
            tfError.clear(); // have to clear the error because the same error messages accumulates
        }
        RCLCPP_INFO(logger, "Transform available!");

        initComplete = true; // costmapCallback will call planPath everytime costmap is received    
    }

private:
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
    
    // flags
    bool continueMapping = false;
    bool costmapReceived = false;
    bool initComplete = false;

    rclcpp_action::Server<AutoSlam>::SharedPtr action_server;
    std::shared_ptr<GoalHandleAutoSlam> goal_handle;

    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        costmapReceived = true;
        mapFrame = msg->header.frame_id;
        if (initComplete && continueMapping) {planPath(msg);} 
    }

    void planPath(const nav2_msgs::msg::Costmap::SharedPtr costmap)
    {
        // get robot pose
        // then get the nearest free cell
        std::vector<Frontiers> frontiersList;

        search.updateMap(costmap);
        auto pose = robotWorldPose();
        geometry_msgs::msg::Pose robotPose;
        robotPose.position.x = pose.transform.translation.x;
        robotPose.position.y = pose.transform.translation.y;
        frontiersList = search.searchFrontiers(robotPose);

        // sort the frontiers
        // pick the frontiers with the lowest cost that is not in the black list

        // if the picked goal is the same as the previous goal
        // check if the robot has made progress towards the goal
        // yes: continue no: put the goal in the black list

        // if the picked goal is new, send the move_base action

        // sort frontiersList based on cost
        std::sort(frontiersList.begin(), frontiersList.end(),
                  [](const Frontiers& f1, const Frontiers& f2) { return f1.cost < f2.cost;});
        search.visualizeFrontiers(frontiersList);

        // picked the frontiers with the lowest cost that is not on the blacklist
        auto goalFrontiers = 
        std::find_if_not(frontiersList.begin(), frontiersList.end(),
                         [this](const Frontiers& f) {return search.goalOnBlacklist(f.centriod);
                         });

        if (goalFrontiers == frontiersList.end())
        {
            RCLCPP_WARN(logger, "No frontiers left to explore");
            continueMapping = false;
            auto result = std::make_shared<AutoSlam::Result>();
            if (!goalFrontiers->points.empty())
            {
                // not all frontiers were cleared
                // all the frontiers left are in the black list
                // map is incomplete
                result->all_frontiers_cleared = false;
            }
            else
            {
                // all frontiers were cleared
                // map is complete
                result->all_frontiers_cleared = true;
            }
            goal_handle->succeed(result); // send the goal to client
            RCLCPP_INFO(logger, "Map sent to client");
            return;
        }

        geometry_msgs::msg::Point goal = goalFrontiers->centriod;
        RCLCPP_INFO(logger, "Picked frontiers:\nCost: %f Centroid: x=%f y=%f",
                            goalFrontiers->cost, goalFrontiers->centriod.x, goalFrontiers->centriod.y);

        bool sameGoal = samePoint(goal, prevGoal);
        prevGoal = goal;

        if (!sameGoal || prevDistanceToGoal > goalFrontiers->min_distance)
        {
            // if not the same goal
            // or same goal but made some progress
            lastProgress = this->now();
            prevDistanceToGoal = goalFrontiers->min_distance;
        }

        if (this->now() - lastProgress > tf2::durationFromSec(progressTimeout))
        {
            search.addToBlacklist(goal);
            return;
        }

        if (sameGoal)
        {
            return; // if same goal, don't need to send goal action
        }
        // at if the code gets here, it means the frontier is new and not on the blacklist

        RCLCPP_INFO(logger, "Sending goal to move_base server");
        auto nav2Goal = nav2_msgs::action::NavigateToPose::Goal();
        nav2Goal.pose.pose.position = goal;
        nav2Goal.pose.pose.orientation.w = 1.;
        nav2Goal.pose.header.frame_id = costmap->header.frame_id;
        nav2Goal.pose.header.stamp = this->now();

        // set result callback
        auto sendGoalOptions = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        sendGoalOptions.result_callback = 
        [this](const NavigationGoalHandle::WrappedResult& result) {reachedGoal(result);};

        // send goal action command
        moveBaseClient->async_send_goal(nav2Goal, sendGoalOptions); // send async to prevent deadlock
    }

    void reachedGoal(const NavigationGoalHandle::WrappedResult& result)
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

    geometry_msgs::msg::TransformStamped robotWorldPose()
    {
        geometry_msgs::msg::TransformStamped transform;
        geometry_msgs::msg::TransformStamped emptyTransform;
        try
        {
            transform = tfBuffer.lookupTransform(mapFrame, robotFrame, this->now(), 100ms);
        }
        catch (tf2::LookupException& e)
        {
            RCLCPP_ERROR(logger, "No transform available error: %s\n", e.what());
            return emptyTransform;
        }
        catch (tf2::ConnectivityException& e)
        {
            RCLCPP_ERROR(logger, "Connectivity error: %s\n", e.what());
            return emptyTransform;
        }
        catch (tf2::ExtrapolationException& e)
        {
            RCLCPP_ERROR(logger, "Extrapolation error: %s\n", e.what());
            return emptyTransform;
        }
        return transform;
    }

    // callback for handling goal
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const AutoSlam::Goal> goal)
    {
        RCLCPP_INFO(logger, "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // callback for handling goal cancellation
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleAutoSlam> goal_handle)
    {
        RCLCPP_INFO(logger, "Received request to cancel goal");
        (void)goal_handle;
        continueMapping = false; // stops costmapCallback from calling planPath
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // callback for handling goal acceptance
    void handle_accepted(const std::shared_ptr<GoalHandleAutoSlam> goal_handle)
    {
        continueMapping = true; // allows costmapCallback to call planPath 
        RCLCPP_INFO(logger, "Starting the mapping process");
        this->goal_handle = goal_handle; // need to store goal handle to send result later (goal_handle->succeed(result))
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousSlamActionServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}