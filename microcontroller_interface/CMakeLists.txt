cmake_minimum_required(VERSION 3.8)
project(microcontroller_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

include_directories(
  /include
)

set(odom_imu_node_dependencies
  rclcpp
  nav_msgs
  sensor_msgs
)

set(keyboard_teleop_dependencies
  rclcpp
  geometry_msgs
)

add_executable(odom_imu_node src/odom_imu_node.cpp)
ament_target_dependencies(odom_imu_node ${odom_imu_node_dependencies})
target_link_libraries(odom_imu_node ${SERIAL_LDFLAGS})
target_link_directories(odom_imu_node PRIVATE ${SERIAL_INCLUDE_DIRS})

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop ${keyboard_teleop_dependencies})

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

INSTALL(TARGETS
  odom_imu_node
  keyboard_teleop
  DESTINATION lib/${PROJECT_NAME}
)

INSTALL(DIRECTORY
  config  
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
