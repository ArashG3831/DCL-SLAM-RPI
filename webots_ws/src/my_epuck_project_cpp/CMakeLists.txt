cmake_minimum_required(VERSION 3.16)
project(my_epuck_project_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_library(LGPIO_LIBRARY NAMES lgpio liblgpio.so.1
  PATHS /lib/aarch64-linux-gnu /usr/lib/aarch64-linux-gnu REQUIRED)

add_library(motor_core
  src/quadrature_decoder.cpp
  src/diffdrive_control_core.cpp
)
target_include_directories(motor_core PUBLIC include)
target_compile_options(motor_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(d500_core
  src/d500_parser.cpp
)
target_include_directories(d500_core PUBLIC include)
target_compile_options(d500_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(real_diffdrive_node_cpp
  src/real_diffdrive_node_cpp.cpp
  src/lgpio_motor_hardware.cpp
)
target_include_directories(real_diffdrive_node_cpp PRIVATE include)
target_compile_options(real_diffdrive_node_cpp PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(real_diffdrive_node_cpp motor_core ${LGPIO_LIBRARY})
ament_target_dependencies(real_diffdrive_node_cpp
  rclcpp geometry_msgs nav_msgs std_msgs tf2_ros)

add_executable(d500_ros2_scan_cpp src/d500_ros2_scan_cpp.cpp)
target_include_directories(d500_ros2_scan_cpp PRIVATE include)
target_compile_options(d500_ros2_scan_cpp PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(d500_ros2_scan_cpp d500_core)
ament_target_dependencies(d500_ros2_scan_cpp rclcpp sensor_msgs)

add_executable(d500_parser_cli src/d500_parser_cli.cpp)
target_include_directories(d500_parser_cli PRIVATE include)
target_compile_options(d500_parser_cli PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(d500_parser_cli d500_core)

add_executable(motor_core_cli src/motor_core_cli.cpp)
target_include_directories(motor_core_cli PRIVATE include)
target_compile_options(motor_core_cli PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(motor_core_cli motor_core)

install(TARGETS motor_core d500_core real_diffdrive_node_cpp d500_ros2_scan_cpp
  d500_parser_cli motor_core_cli
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)
install(DIRECTORY cpp_port_validation/ DESTINATION share/${PROJECT_NAME}/cpp_port_validation)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS
  tools/capture_d500_bytes.py
  tools/cpp_port_benchmark.py
  tools/d500_cpp_live_benchmark.py
  tools/d500_cpp_ros_smoke.py
  tools/generate_d500_golden.py
  tools/generate_motor_golden.py
  tools/hardware_pair_benchmark.py
  tools/motor_backend_benchmark.py
  DESTINATION share/${PROJECT_NAME}/tools)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motor_core test/test_motor_core.cpp)
  target_link_libraries(test_motor_core motor_core)
  target_include_directories(test_motor_core PRIVATE include)

  ament_add_gtest(test_d500_parser test/test_d500_parser.cpp)
  target_link_libraries(test_d500_parser d500_core)
  target_include_directories(test_d500_parser PRIVATE include)

  add_test(NAME test_d500_python_cpp_golden
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_d500_golden.py
      $<TARGET_FILE:d500_parser_cli>)
  add_test(NAME test_motor_python_cpp_golden
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_motor_golden.py
      $<TARGET_FILE:motor_core_cli>)
endif()

ament_package()
