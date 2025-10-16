cmake_minimum_required(VERSION 3.8)
project(ros2_test)

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
find_package(std_msgs REQUIRED)
find_package(OpenVINO REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(communication REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(casadi REQUIRED)

add_executable(test1 src/test1.cpp)
add_executable(test2 src/test2.cpp)
add_executable(test3 src/test3.cpp)
add_executable(test4 src/test4.cpp)
add_executable(mpc_test src/mpc_test.cpp)
add_executable(est_debug src/estimator_debug.cpp)

target_link_libraries(test3 openvino::runtime)
target_link_libraries(test1 yaml-cpp)
target_link_libraries(test4 yaml-cpp pinocchio::pinocchio)
target_link_libraries(mpc_test casadi)
target_link_libraries(est_debug yaml-cpp pinocchio::pinocchio ) 
ament_target_dependencies(mpc_test rclcpp)
ament_target_dependencies(test1 rclcpp)
ament_target_dependencies(test2 rclcpp std_msgs)
ament_target_dependencies(test4 rclcpp)
ament_target_dependencies(est_debug std_msgs rclcpp nav_msgs sensor_msgs communication)

install(
  TARGETS test1 test2 test3 test4 mpc_test est_debug
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
