cmake_minimum_required(VERSION 3.8)
project(action_demo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

# Include generated header files
include_directories(include)

# Add executables for action server and client
add_executable(count_until_server 
  src/count_until_server.cpp
)

ament_target_dependencies(count_until_server 
  rclcpp 
  rclcpp_action 
  action_msgs
  custom_interfaces
)

add_executable(count_until_client 
  src/count_until_client.cpp
)

ament_target_dependencies(count_until_client 
  rclcpp 
  rclcpp_action 
  action_msgs
  custom_interfaces
)

# Add executables for file processing action server and client
add_executable(process_file_server 
  src/process_file_server.cpp
)
ament_target_dependencies(process_file_server 
  rclcpp 
  rclcpp_action 
  action_msgs
  custom_interfaces
)

add_executable(process_file_client 
  src/process_file_client.cpp
)
ament_target_dependencies(process_file_client 
  rclcpp
  rclcpp_action
  action_msgs
  custom_interfaces
)

# Install executables
install(TARGETS
  count_until_server
  count_until_client
  process_file_server
  process_file_client
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
