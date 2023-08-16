cmake_minimum_required(VERSION 3.8)
project(frontier_explorer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mobile_bot_msgs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  visualization_msgs
  tf2
  tf2_ros
  mobile_bot_msgs
)

add_executable(frontier_explorer 
  src/main.cpp 
  src/frontier_explorer.cpp 
  src/frontier_search.cpp
)
ament_target_dependencies(frontier_explorer ${dependencies})

# # compiling action server
# add_executable(run_autoslam_action_server src/frontier_explorer_node.cpp src/frontier_search.cpp)
# target_include_directories(run_autoslam_action_server PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(run_autoslam_action_server PRIVATE "CPP_ACTION_TUTORIALS_BUILDING_DLL")
# ament_target_dependencies(run_autoslam_action_server ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
  frontier_explorer
  # run_autoslam_action_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
