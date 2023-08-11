cmake_minimum_required(VERSION 3.8)
project(keyboard_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(SDL2 REQUIRED)

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

include_directories(${SDL2_INCLUDE_DIRS})
add_executable(start_keyboard_teleop src/keyboard_teleop_node.cpp)
ament_target_dependencies(start_keyboard_teleop rclcpp geometry_msgs)
target_link_libraries(start_keyboard_teleop ${SDL2_LIBRARIES})

install(TARGETS 
  start_keyboard_teleop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
