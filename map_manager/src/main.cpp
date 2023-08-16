#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// #include "mobile_bot_msgs/action/autonomous_slam.hpp"
#include "lifecycle_manager/lifecycle_manager_client.hpp"

#include <filesystem>

using namespace nav2_map_server;
using namespace std::placeholders;
// using AutoSlam = mobile_bot_msgs::action::AutonomousSlam;
// using GoalHandleAutoSlam = rclcpp_action::ClientGoalHandle<AutoSlam>;


// potential parameters
// - free_thresh
// - occupied_thresh
// - map_mode_from_string

// action client callbacks
// goal response callback
// feedback callback
// result callback
// class MasterNode : public rclcpp::Node
// {

// public:
//     MasterNode()
//     : Node("master_node")
//     , logger(this->get_logger())
//     {
        
        
//         client = rclcpp_action::create_client<AutoSlam>(
//             this,
//             "autonomous_slam");
        
//         saveMap();
//     }

// private:
//     rclcpp::Logger logger;
//     MapSaver::SharedPtr map_saver;
//     rclcpp_action::Client<AutoSlam>::SharedPtr client;

//     void saveMap()
//     {
        
//         // set save parameters and map topic
//         SaveParameters save_parameters;
//         std::string map_topic = "global_costmap/costmap";
//         save_parameters.map_file_name = map_file_path;
//         save_parameters.image_format = image_format;
//         save_parameters.free_thresh = 0.25;
//         save_parameters.occupied_thresh = 0.65;
//         save_parameters.mode = map_mode_from_string("trinary");

//         try 
//         {
//             if (map_saver->saveMapTopicToFile(map_topic, save_parameters)) 
//             {
//                 RCLCPP_INFO(logger, "Successfully saved map");
//             } 
//             else 
//             {
//                 RCLCPP_INFO(logger, "Failed to save map");
//             }
//         } 
//         catch (std::exception & e) 
//         {
//             RCLCPP_ERROR(logger, "Unexpected problem appear: %s", e.what());
//             return;
//         }
//     }

//     void callAutoSlam()
//     {
//         RCLCPP_INFO(logger, "Waiting AutoSlam action server");
//         client->wait_for_action_server();

//         auto goal_msg = AutoSlam::Goal();
//         goal_msg.start_map = true;

//         RCLCPP_INFO(this->get_logger(), "Sending goal");

//         // set all the callbacks that can occur after sending a request
//         // goal_response_callback = the response of the server acknowledging the request (comes right after the request is sent)
//         // feedback_callback = the constant feedback message sent by the server
//         // result_callback = the final result of the request
//         auto send_goal_options = rclcpp_action::Client<AutoSlam>::SendGoalOptions();
//         send_goal_options.goal_response_callback =
//         std::bind(&MasterNode::goal_response_callback, this, _1);
//         send_goal_options.feedback_callback =
//         std::bind(&MasterNode::feedback_callback, this, _1, _2);
//         send_goal_options.result_callback =
//         std::bind(&MasterNode::result_callback, this, _1);

//         // send the request
//         client->async_send_goal(goal_msg, send_goal_options);
//     }

//     void goal_response_callback(const GoalHandleAutoSlam::SharedPtr & goal_handle)
//     {
//         if (!goal_handle) {
//         RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//         } else {
//         RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//         }
//     }

//     void feedback_callback(
//         GoalHandleAutoSlam::SharedPtr,
//         const std::shared_ptr<const AutoSlam::Feedback> feedback)
//     {
//         (void)feedback;
//     }

//     void result_callback(const GoalHandleAutoSlam::WrappedResult & result)
//     {
//         // check if how the action went
//         switch (result.code) {
//         case rclcpp_action::ResultCode::SUCCEEDED:
//             break;
//         case rclcpp_action::ResultCode::ABORTED:
//             RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//             return;
//         case rclcpp_action::ResultCode::CANCELED:
//             RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//             return;
//         default:
//             RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//             return;
//         }

//         if (result.result->all_frontiers_cleared)
//         {
//             RCLCPP_INFO(logger, "All frontiers cleared");
//         }
//         else
//         {
//             RCLCPP_INFO(logger, "Not all frontiers cleared\n Restarting the mapping process");
//         }

//     }
// };

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

    // testing
    while (rclcpp::ok())
    {
        RCLCPP_INFO(logger, "Press a/s to start or stop everything\n"
                            "Press d to add Slam node to lifecycle manager\n"
                            "Press q to quit");
        char d = getchar();
        if (d == 'a')
        {
            lifecycle_client->startup();
        }
        else if (d == 's')
        {
            lifecycle_client->shutdown();
        }
        else if (d == 'd')
        {
            lifecycle_client->add_node("async_slam");
        }
        else if (d == 'q')
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
