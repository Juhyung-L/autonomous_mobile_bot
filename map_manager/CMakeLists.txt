cmake_minimum_required(VERSION 3.8)
project(map_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set C++17 for filesystem
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(mobile_bot_msgs REQUIRED)
find_package(lifecycle_manager REQUIRED)
find_package(frontier_explorer REQUIRED)

# debug
# message(STATUS "lifecycle_manager_INCLUDE_DIRS = ${lifecycle_manager_INCLUDE_DIRS}")
# message(STATUS "lifecycle_manager_LIBRARIES = ${lifecycle_manager_LIBRARIES}")

set(dependencies
  rclcpp
  nav2_map_server
  mobile_bot_msgs
  lifecycle_manager
  frontier_explorer
)

include_directories(
  include
  ${lifecycle_manager_INCLUDE_DIRS}
  ${frontier_explorer_INCLUDE_DIRS}
)

add_executable(map_manager src/main.cpp)
ament_target_dependencies(map_manager ${dependencies})
target_link_libraries(map_manager 
  stdc++fs 
  ${lifecycle_manager_LIBRARIES}
  ${frontier_explorer_LIBRARIES}
) # add filesystem to executable

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
  map_manager
  DESTINATION lib/${PROJECT_NAME}
)

# don't install map folder to package share
# because the map in package share gets replaced by the map from the source directory
# if the source directory's map folder is empty, the map in package share gets deleted
# install(DIRECTORY
#   map
#   DESTINATION share/${PROJECT_NAME})

ament_package()
