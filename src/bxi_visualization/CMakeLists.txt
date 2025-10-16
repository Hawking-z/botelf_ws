cmake_minimum_required(VERSION 3.8)
project(bxi_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake REQUIRED)

add_executable(odom2tf src/odom2tf.cpp)
ament_target_dependencies(odom2tf rclcpp nav_msgs tf2_ros std_msgs)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
         DESTINATION share/${PROJECT_NAME})

install(DIRECTORY model
         DESTINATION share/${PROJECT_NAME})

install(PROGRAMS  scripts/rosbag_controller.py
          DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS odom2tf
        DESTINATION lib/${PROJECT_NAME})

ament_package()
