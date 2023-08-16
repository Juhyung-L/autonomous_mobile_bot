cmake_minimum_required(VERSION 3.8)
project(lifecycle_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(mobile_bot_msgs REQUIRED)

include_directories(include)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/lifecycle_manager.cpp
  src/lifecycle_manager_client.cpp
)

set(dependencies
  rclcpp
  nav2_util
  std_srvs
  nav2_msgs
  bondcpp
  diagnostic_updater
  mobile_bot_msgs
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(lifecycle_manager src/main.cpp)
target_link_libraries(lifecycle_manager ${library_name})
ament_target_dependencies(lifecycle_manager ${dependencies})

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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
  lifecycle_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include) # enables lifecycle_manager_INCLUDE_DIRS
ament_export_libraries(${library_name}) # enables lifecycle_manager_LIBRARIES
ament_export_dependencies(${dependencies})
# prevents the need to include the dependencies of this package
# (such as rclcpp, std_srvs, and nav2_msgs)
# in the other package that is using its shared library

ament_package()
