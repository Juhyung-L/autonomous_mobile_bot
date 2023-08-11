#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dummy_node");
    auto lifecycle_client = std::make_shared<nav2_lifecycle_manager::LifecycleManagerClient>(
        "lifecycle_manager_navigation", node);
    nav2_lifecycle_manager::SystemStatus status = lifecycle_client->is_active(1s);
    switch(status)
    {
        case nav2_lifecycle_manager::SystemStatus::ACTIVE:
            RCLCPP_INFO(node->get_logger(), "System active");
            break;
        case nav2_lifecycle_manager::SystemStatus::INACTIVE:
            RCLCPP_INFO(node->get_logger(), "System inactive");
            break;
        case nav2_lifecycle_manager::SystemStatus::TIMEOUT:
            RCLCPP_INFO(node->get_logger(), "System timed out");
            break;
        default:
            RCLCPP_INFO(node->get_logger(), "Unknown response from server");
            break;
    }
    // lifecycle_client->startup(1s);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}