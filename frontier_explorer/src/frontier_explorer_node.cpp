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

class Explorer : public rclcpp::Node
{
    using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
public:
    Explorer()
    : Node("frontier_explorer_node")
    , tfBuffer(this->get_clock())
    , tfListener(tfBuffer)
    , search(*this)
    { 
        this->declare_parameter<std::string>("robot_frame", "base_footprint");
        this->declare_parameter<int>("progress_timeout", 10);

        this->get_parameter("robot_frame", robotFrame);
        this->get_parameter("progress_timeout", progressTimeout);
        // time required for a goal to be considered unreachable and be put into the blacklist

        costmapSub = this->create_subscription<nav2_msgs::msg::Costmap>(
            "global_costmap/costmap_raw", 10, std::bind(&Explorer::costmapCallback, this, _1));
        
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
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        costmapReceived = true;
        mapFrame = msg->header.frame_id;
        if (initComplete) {planPath(msg);} 
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
        moveBaseClient->async_send_goal(nav2Goal, sendGoalOptions);
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Explorer>());
    rclcpp::shutdown();
    return 0;
}
