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
find_package(nav2_util)
find_package(rclcpp_lifecycle)
find_package(lifecycle_msgs)

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
  nav2_util
  rclcpp_lifecycle
  lifecycle_msgs
)

# building client (library)
add_library(frontier_explorer_client SHARED
  src/frontier_explorer_client.cpp)
target_include_directories(frontier_explorer_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(frontier_explorer_client
  PRIVATE "NAV2_MSGS_CPP_BUILDING_DLL")
ament_target_dependencies(frontier_explorer_client
  rclcpp
  rclcpp_action
  nav2_msgs)

# building server (executable)
add_executable(frontier_explorer_server
  src/frontier_explorer_server_node.cpp 
  src/frontier_explorer_server.cpp 
  src/frontier_search.cpp)
target_include_directories(frontier_explorer_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(frontier_explorer_server
  PRIVATE "NAV2_MSGS_CPP_BUILDING_DLL")
ament_target_dependencies(frontier_explorer_server ${dependencies})


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

install(DIRECTORY include/ DESTINATION include/)

install(TARGETS
  frontier_explorer_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include) # enables frontier_explorer_INCLUDE_DIRS
ament_export_libraries(frontier_explorer_client) # enables frontier_explorer_LIBRARIES
ament_export_dependencies(
  rclcpp
  rclcpp_action
  mobile_bot_msgs
)
# prevents the need to include the dependencies of this package
# in the other package that is using this shared library

install(TARGETS 
  frontier_explorer_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
