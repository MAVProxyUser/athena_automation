cmake_minimum_required(VERSION 3.5)
project(dm_planner)

set(CMAKE_BUILD_TYPE DEBUG)

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
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    nav2_util
    nav2_msgs
    nav_msgs
    geometry_msgs
    nav2_common
    tf2_ros
    nav2_costmap_2d
    nav2_core
    pluginlib
)

add_library(dmp SHARED src/distance_map_planner.cpp src/graph_search.cpp)

add_library(dm_planner SHARED src/distance_map_planner_node.cpp)
target_link_libraries(dm_planner dmp)
ament_target_dependencies(dm_planner ${dependencies})

install(TARGETS dmp dm_planner 
    DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include/
)

install(FILES global_planner_plugin.xml
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(dmp dm_planner)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

ament_package()
