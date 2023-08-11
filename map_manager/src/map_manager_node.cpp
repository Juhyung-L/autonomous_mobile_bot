#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "mobile_bot_msgs/action/autonomous_slam.hpp"

#include <filesystem>

using namespace nav2_map_server;
using namespace std::placeholders;

// potential parameters
// - free_thresh
// - occupied_thresh
// - map_mode_from_string

// action client callbacks
// goal response callback
// feedback callback
// result callback
class MapManager : public rclcpp::Node
{
    using AutoSlam = mobile_bot_msgs::action::AutonomousSlam;
    using GoalHandleAutoSlam = rclcpp_action::ClientGoalHandle<AutoSlam>;

public:
    MapManager()
    : Node("map_manager")
    , logger(this->get_logger())
    {
        map_saver = std::make_shared<nav2_map_server::MapSaver>();
        map_saver->on_configure(rclcpp_lifecycle::State());
        
        client = rclcpp_action::create_client<AutoSlam>(
            this,
            "autonomous_slam");
        
        saveMap();
    }

private:
    rclcpp::Logger logger;
    MapSaver::SharedPtr map_saver;
    rclcpp_action::Client<AutoSlam>::SharedPtr client;

    void saveMap()
    {
        // try to get path to map_manager package
        std::string package_path;
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
        map_file_path = package_path + "/map/map";

        std::filesystem::path fs_map_file_path(map_file_path + "." + image_format);
        if (!std::filesystem::exists(fs_map_file_path))
        {
            RCLCPP_INFO(logger, "Map does not exist");
            // call the mapping action
            // save the map when mapping is completed
            callAutoSlam();
            // block until the result is received
        }
        else
        {
            RCLCPP_INFO(logger, "Map exists");
        }

        // set save parameters and map topic
        SaveParameters save_parameters;
        std::string map_topic = "global_costmap/costmap";
        save_parameters.map_file_name = map_file_path;
        save_parameters.image_format = image_format;
        save_parameters.free_thresh = 0.25;
        save_parameters.occupied_thresh = 0.65;
        save_parameters.mode = map_mode_from_string("trinary");

        try 
        {
            if (map_saver->saveMapTopicToFile(map_topic, save_parameters)) 
            {
                RCLCPP_INFO(logger, "Successfully saved map");
            } 
            else 
            {
                RCLCPP_INFO(logger, "Failed to save map");
            }
        } 
        catch (std::exception & e) 
        {
            RCLCPP_ERROR(logger, "Unexpected problem appear: %s", e.what());
            return;
        }
    }

    void callAutoSlam()
    {
        RCLCPP_INFO(logger, "Waiting AutoSlam action server");
        client->wait_for_action_server();

        auto goal_msg = AutoSlam::Goal();
        goal_msg.start_map = true;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        // set all the callbacks that can occur after sending a request
        // goal_response_callback = the response of the server acknowledging the request (comes right after the request is sent)
        // feedback_callback = the constant feedback message sent by the server
        // result_callback = the final result of the request
        auto send_goal_options = rclcpp_action::Client<AutoSlam>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&MapManager::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&MapManager::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&MapManager::result_callback, this, _1);

        // send the request
        client->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleAutoSlam::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleAutoSlam::SharedPtr,
        const std::shared_ptr<const AutoSlam::Feedback> feedback)
    {
        (void)feedback;
    }

    void result_callback(const GoalHandleAutoSlam::WrappedResult & result)
    {
        // check if how the action went
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        if (result.result->all_frontiers_cleared)
        {
            RCLCPP_INFO(logger, "All frontiers cleared");
        }
        else
        {
            RCLCPP_INFO(logger, "Not all frontiers cleared\n Restarting the mapping process");
        }

    }
};

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapManager>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}