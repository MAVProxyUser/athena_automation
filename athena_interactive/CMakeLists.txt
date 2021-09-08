cmake_minimum_required(VERSION 3.5)
project(interactive)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(athena_utils REQUIRED)

find_package(interaction_msgs REQUIRED)
find_package(automation_msgs REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(cascade_lifecycle_msgs REQUIRED)
find_package(ception_msgs REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/include
  ${athena_utils_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  nav_msgs
  std_msgs
  std_srvs
  automation_msgs
  motion_msgs
  interaction_msgs
  cascade_lifecycle_msgs
  ception_msgs
  athena_utils
  OpenCV
  tf2_ros
)

add_executable(interactive_node src/interactive.cpp src/speaker_client.cpp)
target_include_directories(interactive_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(interactive_node ${dependencies})

# test
add_executable(client_test src/client_test.cpp)
ament_target_dependencies(client_test "rclcpp" ${dependencies})

install(TARGETS interactive_node client_test
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
