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

std::vector<const char*> helper_msgs{
    "Map file detected on machine.\n Press {L} to load the saved map file or {R} to generate a new map file.",
    "No map file detected on machine.\n Press {R} to generate a map file"
};

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

    frontier_explorer::FrontierExplorerClient frontier_explorer_client(node);

    // testing
    auto handle_user_input = [&]()
    {
        while (rclcpp::ok())
        {
        // get the current state of lifecycle nodes
        lifecycle_manager::SystemStatus lifecycle_status = lifecycle_client->is_active();

        // nodes are active
        if (lifecycle_status == lifecycle_manager::SystemStatus::ACTIVE)
        {
            bool valid = frontier_explorer_client.action_status.valid();
            
            // didn't send action goal yet
            if (!valid)
            {
                RCLCPP_INFO(logger, "\n"
                                    "Nav2 is running\n"
                                    "Press {P} to pause\n"
                                    "Press {F} to send goal to frontier explorer action server\n"
                                    "Press {Q} to quit");
                char d = getchar();
                
                if (d == 'p')
                {
                    lifecycle_client->pause();
                }
                else if (d == 'f')
                {
                    frontier_explorer_client.sendGoal(); // sets the action_status future
                }
                else if (d == 'q')
                {
                    lifecycle_client->shutdown();
                    break;
                }
                else
                {
                    RCLCPP_INFO(logger, "Invalid input");
                }
            }
            else if (valid) // frontier explorer action is running
            {
                if (frontier_explorer_client.action_status.get()->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
                {
                    RCLCPP_INFO(logger, "\n"
                                        "Nav2 is running\n"
                                        "Frontier explorer action is running\n"
                                        "Press {P} to pause\n"
                                        "Press {Q} to quit");
                    char d = getchar();
                    
                    if (d == 'p')
                    {
                        lifecycle_client->pause();
                    }
                    else if (d == 'q')
                    {
                        lifecycle_client->shutdown();
                        break;
                    }
                    else
                    {
                        RCLCPP_INFO(logger, "Invalid input");
                    }
                }
                else if (frontier_explorer_client.action_status.get()->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
                {
                    RCLCPP_INFO(logger, "Code amcl part now");
                }
            }
        }
        else if (lifecycle_status == lifecycle_manager::SystemStatus::INACTIVE)
        {
            RCLCPP_INFO(logger, "\n"
                                "Nav2 is not running\n"
                                "Press {R} to resume\n"
                                "Press {Q} to quit");
            char d = getchar();
            
            if (d == 'r')
            {
                lifecycle_client->resume();
            }
            else if (d == 'q')
            {
                lifecycle_client->shutdown();
                break;
            }
            else
            {
                RCLCPP_INFO(logger, "Invalid input");
            }
        }
        else if (lifecycle_status == lifecycle_manager::SystemStatus::TIMEOUT)
        {
            RCLCPP_INFO(logger, "Lifecycle manager timeout...");
        }

        // Clear the cin buffer
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::this_thread::sleep_for(100ms);
        }
    };
    
    std::thread user_input_thread(handle_user_input);

    rclcpp::spin(node);
    
    if (user_input_thread.joinable())
    {
        user_input_thread.join();
    }
    
    rclcpp::shutdown();

    return 0;
}
