#include <thread>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/explore_frontier.hpp"

using namespace std::placeholders;

// potential parameters
// - free_thresh
// - occupied_thresh
// - map_mode_from_string

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    
    auto logger = rclcpp::get_logger("main_app");

    auto node = std::make_shared<rclcpp::Node>("main_app");
    auto lifecycle_client =
        std::make_shared<nav2_lifecycle_manager::LifecycleManagerClient>(
            "lifecycle_manager_navigation", node);

    auto handle_user_input = [&]()
    {
        while (rclcpp::ok())
        {
            RCLCPP_INFO(logger, "\n"
                                "Press a/s to startup or reset lifecycle-managed nodes\n"
                                "Press z/x to resume or pause lifecycle-managed nodes\n"
                                "Press q to shutdown and quit");
            char d = getchar();
            
            if (d == 'a')
            {
                lifecycle_client->startup();
            }
            else if (d == 's')
            {
                lifecycle_client->reset();
            }
            else if (d == 'z')
            {
                lifecycle_client->resume();
            }
            else if (d == 'x')
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
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
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