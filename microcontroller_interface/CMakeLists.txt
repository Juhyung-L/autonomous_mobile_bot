cmake_minimum_required(VERSION 3.8)
project(microcontroller_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

include_directories(
  /include
)

set(dependencies
  rclcpp
  geometry_msgs
  tf2_ros
)

add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node ${dependencies})
target_link_libraries(imu_node ${SERIAL_LDFLAGS})
target_link_directories(imu_node PRIVATE ${SERIAL_INCLUDE_DIRS})

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
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
