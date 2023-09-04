#include <filesystem>
#include <thread>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "lifecycle_manager/lifecycle_manager_client.hpp"
#include "mobile_bot_msgs/action/explore_frontier.hpp"
#include "frontier_explorer/frontier_explorer_client.hpp"

using namespace nav2_map_server;
using namespace std::placeholders;
using namespace std::chrono_literals;

// potential parameters
// - free_thresh
// - occupied_thresh
// - map_mode_from_string

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    
    auto logger = rclcpp::get_logger("main_app");
    
    // remapping route (TODO: make map loading route)
    // start nav2
    auto node = std::make_shared<rclcpp::Node>("main_app");
    auto lifecycle_client =
        std::make_shared<lifecycle_manager::LifecycleManagerClient>(
            "lifecycle_manager_navigation", node);
    auto save_map_client = node->create_client<nav2_msgs::srv::SaveMap>(
        "map_saver_server/save_map");
    frontier_explorer::FrontierExplorerClient frontier_explorer_client(node);

    auto start_mapping = [&]()
    {
        RCLCPP_INFO(logger, "Starting autonomous mapping...");
        lifecycle_client->add_node("async_slam");
        lifecycle_client->add_node("frontier_explorer_server");
        lifecycle_client->add_node("map_saver_server");
        lifecycle_client->startup();
        while (lifecycle_client->is_active() != lifecycle_manager::SystemStatus::ACTIVE
                && rclcpp::ok())
        {
            std::this_thread::sleep_for(100ms);
        }
        frontier_explorer_client.sendGoal();
    };

    auto event_loop = [&]()
    {
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
        if (false || std::filesystem::exists(fs_map_file_path)) // map file exists
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

        if (frontier_explorer_client.action_future.valid())
        {
            while (rclcpp::ok()
                   && frontier_explorer_client.action_future.get()->get_status() != rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
            {
                lifecycle_manager::SystemStatus lifecycle_status = lifecycle_client->is_active();
                if (lifecycle_status == lifecycle_manager::SystemStatus::ACTIVE)
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
                else if (lifecycle_status == lifecycle_manager::SystemStatus::INACTIVE)
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
                else if (lifecycle_status == lifecycle_manager::SystemStatus::TIMEOUT)
                {
                    RCLCPP_INFO(logger, "lifecycle manager timedout");
                }
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }

            if (frontier_explorer_client.action_future.get()->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
            {
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
        }

        // load map from file
        lifecycle_client->pause();
        lifecycle_client->remove_node("async_slam");
        lifecycle_client->remove_node("frontier_explorer_server");
        lifecycle_client->remove_node("map_saver_server");

        lifecycle_client->add_node("map_server");
        lifecycle_client->add_node("amcl");
        lifecycle_client->resume();

        // start amcl
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
