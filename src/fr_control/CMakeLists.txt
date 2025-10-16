cmake_minimum_required(VERSION 3.8)
project(fr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(communication REQUIRED)
# find_package(debug_msgs REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# find_package(std_msgs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Observation.msg"
#   "msg/Action.msg"
#   DEPENDENCIES std_msgs
# )

# include_directories(
#   include
#   ${rclcpp_INCLUDE_DIRS}
# )
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(TARGET_DEPENDENCE
  ament_cmake
  rclcpp
  nav_msgs
  sensor_msgs
  # debug_msgs
  std_msgs
  communication
)

# file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
set(
  RL_CONTROLLER_SRC
  src/main.cpp
  src/control/RobotController.cpp
  src/utils/math_tools.cpp
)

add_executable(${PROJECT_NAME} ${RL_CONTROLLER_SRC})
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(${PROJECT_NAME} openvino::runtime)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

ament_target_dependencies(${PROJECT_NAME} ${TARGET_DEPENDENCE})
# rosidl_target_interfaces(${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
         DESTINATION share/${PROJECT_NAME})

ament_package()
