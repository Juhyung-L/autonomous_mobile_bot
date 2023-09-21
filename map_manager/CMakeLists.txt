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
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(frontier_explorer REQUIRED)

# debug
# message(STATUS "nav2_lifecycle_manager_INCLUDE_DIRS = ${nav2_lifecycle_manager_INCLUDE_DIRS}")
# message(STATUS "nav2_lifecycle_manager_LIBRARIES = ${nav2_lifecycle_manager_LIBRARIES}")

set(dependencies
  rclcpp
  rclcpp_action
  ament_index_cpp
  geometry_msgs
  nav2_map_server
  nav2_msgs
  nav2_lifecycle_manager
  frontier_explorer
)

include_directories(
  include
  # ${nav2_lifecycle_manager_INCLUDE_DIRS}
  # ${frontier_explorer_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)
ament_target_dependencies(main ${dependencies})
target_link_libraries(main 
  stdc++fs 
  # ${nav2_lifecycle_manager_LIBRARIES}
  # ${frontier_explorer_LIBRARIES}
) # add filesystem to executable

add_executable(test_lifecycle src/test_lifecycle.cpp)
ament_target_dependencies(test_lifecycle ${dependencies})

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
  main
  test_lifecycle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
