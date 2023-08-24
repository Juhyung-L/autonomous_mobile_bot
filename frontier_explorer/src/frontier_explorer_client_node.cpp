#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "frontier_explorer/frontier_explorer_client.hpp"

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<frontier_explorer_client::FrontierExplorerClient>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}