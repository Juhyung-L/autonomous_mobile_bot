#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "frontier_explorer/frontier_explorer.hpp"

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<frontier_explorer::FrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}