cmake_minimum_required(VERSION 3.8)
project(my_epuck_frontier_candidates)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(frontier_exploration_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_epuck_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(frontier_search_core
  src/candidate_utils.cpp
)
target_include_directories(frontier_search_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(frontier_search_core
  frontier_exploration_ros2::frontier_exploration_ros2_core
)
ament_target_dependencies(frontier_search_core
  frontier_exploration_ros2
  geometry_msgs
  nav_msgs
)

add_executable(frontier_candidate_generator
  src/frontier_candidate_generator.cpp
)
target_include_directories(frontier_candidate_generator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(frontier_candidate_generator
  frontier_search_core
)
ament_target_dependencies(frontier_candidate_generator
  geometry_msgs
  my_epuck_interfaces
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  visualization_msgs
)

install(
  TARGETS frontier_search_core frontier_candidate_generator
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
