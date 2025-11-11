cmake_minimum_required(VERSION 3.8)
project(bxi_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(communication REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# utils
add_executable(keyboard_node  src/utils/keyboard_node.cpp)
add_executable(rc_node src/utils/remote_controller_node.cpp)

# control
set(
  RL_CONTROLLER_SRC
  src/rl_main.cpp
  src/control/rl_controller.cpp
  src/utils/math_tools.cpp
)

add_executable(
  rl_controller
  ${RL_CONTROLLER_SRC}
)

target_link_libraries(rc_node yaml-cpp)
target_link_libraries(keyboard_node yaml-cpp)

target_link_libraries(rl_controller yaml-cpp openvino::runtime Eigen3::Eigen)

ament_target_dependencies(keyboard_node rclcpp std_msgs communication)
ament_target_dependencies(rc_node rclcpp std_msgs communication)
ament_target_dependencies(rl_controller std_msgs rclcpp nav_msgs sensor_msgs communication)

# install executable
install(
  TARGETS  rc_node keyboard_node  rl_controller
  DESTINATION lib/${PROJECT_NAME}
)
# install launch
install(DIRECTORY launch 
        DESTINATION share/${PROJECT_NAME})

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
