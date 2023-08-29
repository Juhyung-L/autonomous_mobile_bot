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


#include <iostream>
#include <thread>

void userInputThread(bool& running, std::string& input) {
    while (running) {
        if (std::cin.peek() != EOF) {
            std::getline(std::cin, input);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to avoid high CPU usage
    }
}

int main() {
    bool running = true;
    std::string input;
    
    std::thread inputThread(userInputThread, std::ref(running), std::ref(input));
    
    while (running) {
        // Your other program logic here
        
        // Check if user input is available
        if (!input.empty()) {
            std::cout << "User input: " << input << std::endl;
            input.clear(); // Clear the input buffer
        }
        
        // Continue with other program logic
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Avoid high CPU usage
    }
    
    inputThread.join(); // Wait for the input thread to finish
    
    return 0;
}
