#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_srvs/srv/empty.hpp"

#include "lifecycle_manager/lifecycle_manager_client.hpp"
#include "mobile_bot_msgs/action/explore_frontier.hpp"

#include <filesystem>

using namespace nav2_map_server;
using namespace std::placeholders;


// potential parameters
// - free_thresh
// - occupied_thresh
// - map_mode_from_string

std::vector<const char*> helper_msgs{
    "Map file detected on machine.\n Press {L} to load the saved map file or {R} to generate a new map file.",
    "No map file detected on machine.\n Press {R} to generate a map file"
};

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    
    auto logger = rclcpp::get_logger("main_app");
    // uint8_t helper_index;

    // // check if map is saved
    // std::string package_path;
    // std::string map_file_path;
    // std::string image_format("png");
    // try
    // {
    //     package_path = ament_index_cpp::get_package_share_directory("map_manager");
    // }
    // catch (const std::runtime_error& e)
    // {
    //     RCLCPP_ERROR(logger, "Failed to get package share directory: %s", e.what());
    //     return -1;
    // }
    // map_file_path = package_path + "/map/map";

    // std::filesystem::path fs_map_file_path(map_file_path + "." + image_format);
    // if (std::filesystem::exists(fs_map_file_path)) // map file exists
    // {
    //     helper_index = 0;
    // }
    // else // map file doesn't exist
    // {
    //     helper_index = 1;
    // }

    // RCLCPP_INFO(logger, helper_msgs[helper_index]);
    // char c = getchar(); // block until user input
    // if (helper_index == 0)
    // {
    //     if (c == 'L')
    //     {
    //         RCLCPP_INFO(logger, "Loading saved map...");
    //     }
    //     else if (c== 'R')
    //     {
    //         RCLCPP_INFO(logger, "Calling autonomous slam service...");
    //     }
    //     else
    //     {
    //         RCLCPP_ERROR(logger, "Invalid input");
    //         rclcpp::shutdown();
    //         return -1;
    //     }
    // }
    // else if (helper_index == 1)
    // {
    //     if (c == 'R')
    //     {
    //         RCLCPP_INFO(logger, "Calling autonomous mapping service...");
    //     }
    //     else
    //     {
    //         RCLCPP_ERROR(logger, "Invalid input");
    //         rclcpp::shutdown();
    //         return -1;
    //     }
    // }

    // remapping route (TODO: make map loading route)
    // start nav2
    auto node = std::make_shared<rclcpp::Node>("main_app");
    auto lifecycle_client =
        std::make_shared<lifecycle_manager::LifecycleManagerClient>(
            "lifecycle_manager_navigation", node);
    auto frontier_explorer_client = node->create_client<std_srvs::srv::Empty>("explore_frontier_send_goal");

    // auto action_client = rclcpp_action::create_client<ExploreFrontier>(
    //     this, ""
    // )

    // testing
    while (rclcpp::ok())
    {
        RCLCPP_INFO(logger, "\n"
                            "Press q/w/e to startup, shutdown, reset everything\n"
                            "Press a/s to resume or pause everything\n"
                            "Press f to send goal to frontier explorer action server\n"
                            "Press d to quit");
        char d = getchar();
        
        if (d == 'q')
        {
            lifecycle_client->startup();
        }
        else if (d == 'w')
        {
            lifecycle_client->shutdown();
        }
        else if (d == 'e')
        {
            lifecycle_client->reset();
        }
        else if (d == 'a')
        {
            lifecycle_client->resume();
        }
        else if (d == 's')
        {
            lifecycle_client->pause();
        }
        else if (d == 'f')
        {
            frontier_explorer_client->wait_for_service();
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = frontier_explorer_client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(logger, "Successfully sent request to frontier_explorer server");
            }
            else
            {
                RCLCPP_ERROR(logger, "Failed to send request to frontier_explorer server");
            }
        }
        else if (d == 'd')
        {
            break;
        }
        else
        {
            RCLCPP_INFO(logger, "Invalid input");
        }
    }
    
    // start slam
    // start autonomous mapping
    rclcpp::shutdown();
    return 0;
}
