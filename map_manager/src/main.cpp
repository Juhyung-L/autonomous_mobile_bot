#include <filesystem>
#include <thread>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "frontier_explorer/frontier_explorer_client.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

// potential parameters
// - free_thresh
// - occupied_thresh
// - map_mode_from_string

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    
    // remapping route (TODO: make map loading route)
    // start nav2
    auto node = std::make_shared<rclcpp::Node>("main_app");
    auto logger = node->get_logger();
    auto lifecycle_client =
        std::make_shared<nav2_lifecycle_manager::LifecycleManagerClient>(
            "lifecycle_manager", node);
    auto save_map_client = node->create_client<nav2_msgs::srv::SaveMap>(
        "map_saver_server/save_map");
    auto frontier_explorer_client = 
        std::make_shared<frontier_explorer::FrontierExplorerClient>(node);
    auto amcl_initial_pose_pub = 
        node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", rclcpp::SystemDefaultsQoS());
    auto set_yaml_filename_parameter_client = 
        node->create_client<rcl_interfaces::srv::SetParameters>(
            "/map_server/set_parameters");

    auto start_mapping = [&]()
    {
        RCLCPP_INFO(logger, "Starting autonomous mapping...");
        lifecycle_client->add_node({"async_slam", 
            "frontier_explorer_server", 
            "map_saver_server"});
        lifecycle_client->startup();
        while (lifecycle_client->is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE
                && rclcpp::ok())
        {
            std::this_thread::sleep_for(100ms);
        }
        frontier_explorer_client->sendGoal(); // blocks until the ClientGoalHandle (GoalHandleExploreFrontier) future is set
    };

    auto event_loop = [&]()
    {
        // add all necessary node for the nav2 stack to function
        lifecycle_client->add_node({"controller_server",
            "smoother_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
            "waypoint_follower",
            "velocity_smoother",});
        
        // check if map is saved
        std::string package_path;
        std::string map_folder_path;
        std::string map_file_path;
        std::string image_format("png");
        try
        {
            package_path = ament_index_cpp::get_package_share_directory("map_manager");
        }
        catch (const std::runtime_error& e)
        {
            RCLCPP_ERROR(logger, "Failed to get package share directory: %s", e.what());
            return;
        }
        map_folder_path = package_path + "/map";
        if (!std::filesystem::exists(map_folder_path))
        {
            std::filesystem::create_directory(map_folder_path);
        }

        map_file_path = map_folder_path + "/map";

        std::filesystem::path fs_map_file_path(map_file_path + "." + image_format);
        if (std::filesystem::exists(fs_map_file_path)) // map file exists
        {
            RCLCPP_INFO(logger, "\n"
                                "Map file detected on machine\n"
                                "Press {L} to load the saved map file\n"
                                "Press {R} to generate a new map file");

            while (rclcpp::ok())
            {
                char c = getchar();
                if (c == 'l')
                {
                    RCLCPP_INFO(logger, "Loading saved map...");
                    break;
                }
                else if (c== 'r')
                {
                    start_mapping();
                    break;
                }
            }
        }
        else // map file doesn't exist
        {
            RCLCPP_INFO(logger, "\n"
                                "No map file detected on machine\n"
                                "Press {R} to generate a map file");

            while (rclcpp::ok())
            {
                char c = getchar();
                if (c == 'r')
                {
                    start_mapping();
                    break;
                }
            }
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        auto explore_frontier_goal_handle_future = frontier_explorer_client->getGoalHandleFuture();
        if (explore_frontier_goal_handle_future.valid())
        {
            auto explore_frontier_goal_handle = explore_frontier_goal_handle_future.get();
            while (rclcpp::ok() && 
                   (explore_frontier_goal_handle->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING || 
                    explore_frontier_goal_handle->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED))
            {
                nav2_lifecycle_manager::SystemStatus lifecycle_status = lifecycle_client->is_active();
                if (lifecycle_status == nav2_lifecycle_manager::SystemStatus::ACTIVE)
                {
                    RCLCPP_INFO(logger, "\n"
                                        "Mapping in progress...\n"
                                        "Press {P} to pause\n"
                                        "Press {Q} to quit");
                    char c = getchar();
                    if (c == 'p')
                    {
                        lifecycle_client->pause();
                    }
                    else if (c == 'q')
                    {
                        lifecycle_client->shutdown();
                        break;
                    }
                }
                else if (lifecycle_status == nav2_lifecycle_manager::SystemStatus::INACTIVE)
                {
                    RCLCPP_INFO(logger, "\n"
                                        "Mapping in paused\n"
                                        "Press {R} to resume\n"
                                        "Press {Q} to quit");
                    char c = getchar();
                    if (c == 'r')
                    {
                        lifecycle_client->resume();
                    }
                    else if (c == 'q')
                    {
                        lifecycle_client->shutdown();
                        break;
                    }
                }
                else if (lifecycle_status == nav2_lifecycle_manager::SystemStatus::TIMEOUT)
                {
                    RCLCPP_INFO(logger, "lifecycle manager timedout");
                }
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }

            if (explore_frontier_goal_handle_future.get()->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
            {
                // save map and send end pose to amcl node

                // the amcl node needs a PoseWithCovarianceStamped, 
                // but the end pose fron the frontier_explore_client is a PoseStamped
                // TODO need to set the global frame of amcl node for the message to be accepted
                geometry_msgs::msg::PoseStamped end_pose;
                geometry_msgs::msg::PoseWithCovarianceStamped pose_to_send;
                if (frontier_explorer_client->getEndPose(end_pose))
                {
                    pose_to_send.header = end_pose.header;
                    pose_to_send.pose.pose = end_pose.pose;
                    amcl_initial_pose_pub->publish(pose_to_send);
                }
                else
                {
                    RCLCPP_ERROR(logger, "Could not get robot end pose");
                    return;
                }

                RCLCPP_INFO(logger, "Saving map...");
                auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
                request->free_thresh = 0.25;
                request->occupied_thresh = 0.65;
                request->map_url = map_file_path;
                request->image_format = image_format;
                request->map_mode = "trinary";
                request->map_topic = "/global_costmap/costmap";
                save_map_client->wait_for_service();
                RCLCPP_INFO(logger, "Sending async request");
                auto result = save_map_client->async_send_request(request);
            }
            else
            {
                RCLCPP_ERROR(logger, "Mapping did not suceed.");
                return;
            }
        }

        // load map from file
        lifecycle_client->reset();
        lifecycle_client->remove_node({"async_slam", 
            "frontier_explorer_server", 
            "map_saver_server"});

        lifecycle_client->add_node({"map_server", "amcl"});
        
        // set "yaml_filename" parameter of map_server
        RCLCPP_INFO(logger, "Setting yaml_filename parameter of map_server");
        auto parameter = rcl_interfaces::msg::Parameter();
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        parameter.name = "yaml_filename";
        parameter.value.type = 4; // string type
        parameter.value.string_value = map_file_path + ".yaml";
        request->parameters.push_back(parameter);

        RCLCPP_INFO(logger, "Waiting for service");
        set_yaml_filename_parameter_client->wait_for_service();
        auto set_yaml_filename_future = set_yaml_filename_parameter_client->async_send_request(request);
        
        // calling wait() when valid() returns false leads to undefined behavior
        // so wait until valid is true
        while (!set_yaml_filename_future.valid()) {std::this_thread::sleep_for(100ms);}
        RCLCPP_INFO(logger, "Waiting for setting parameter to complete");
        set_yaml_filename_future.wait();
        lifecycle_client->startup();
    };
    
    std::thread user_input_thread(event_loop);
    rclcpp::spin(node);
    if (user_input_thread.joinable())
    {
        user_input_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
