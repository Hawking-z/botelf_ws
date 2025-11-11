
#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <communication/msg/touch_sensor.hpp>
#include <communication/srv/robot_reset.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <communication/msg/actuator_cmds.hpp>
#include <communication/msg/motion_commands.hpp>

#include <communication/srv/simulation_reset.hpp>

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <eigen3/Eigen/Dense>
#include <openvino/openvino.hpp>

#include <deque>
#include <unordered_map>
#include <cmath>
#include <thread>
#include <chrono>

#include <std_msgs/msg/float32_multi_array.hpp>

#include "utils/cfgutils.h"
#include "utils/ovutils.h"
#include "utils/math_tools.h"
#include "utils/scheduler.h"

using namespace rclcpp;
using namespace std::chrono_literals;


class RobotController : public Node
{
public:
    RobotController();
    ~RobotController();

    void init();

private:
    
    void reset_robot(int reset_step, bool release);
    void callSimulationResetService();

    void init_robot_pos();

    void update_obs();
    void infer_action();

    void load_config(std::string config_path);

    void infer_loop();
    void pd_controller_loop();
    void main_thread();
    // ros2 msgs
    void commands_callback(const communication::msg::MotionCommands::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void touch_callback(const communication::msg::TouchSensor::SharedPtr msg);
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
    std::string topic_prefix_;
    
    std::thread infer_loop_thread_;
    std::thread pd_controller_thread_;
    std::thread main_thread_;

    //Mutexes
    std::mutex motion_commands_mutex_;
    std::mutex imu_mutex_;
    std::mutex joint_mutex_;
    std::mutex touch_mutex_;
    std::mutex odom_mutex_;

    // TODO
    bool run_ = false;
    bool error_flag_ = false;
    int error_count = 0;
    bool pd_controller_flag_ = false;

    // --------通过yaml初始化--------
    const std::string dev = "CPU";
    
    // 初始化
    GaitScheduler scheduler;
    cfgutils::RobotConfig robot_cfg_;
    std::unique_ptr<cfgutils::ObsAssembler> obs_assembler_ptr_;
    std::unique_ptr<ovutils::OVModelIO> ov_model_ptr_;
    std::unordered_map<std::string, cfgutils::DofParam> joints_;
    
    std::vector<std::string> command_order_;

    Eigen::Vector3f init_base_pos_ = Eigen::Vector3f::Zero();
    Eigen::Vector4f init_base_quat_ = Eigen::Vector4f::Zero(); // xyzw
    
    bool with_arm = true;
    bool reset_pose = false;
    // 输入输出
    std::unordered_map<std::string, Eigen::VectorXf> obs_dict_;
    Eigen::VectorXf action_output_;
    // ------------------------------------
    //Sensor data & Commands
    communication::msg::MotionCommands::SharedPtr motion_commands_msg_;
    sensor_msgs::msg::Imu::SharedPtr imu_msg_;
    sensor_msgs::msg::JointState::SharedPtr joint_state_msg_;
    communication::msg::TouchSensor::SharedPtr touch_sensor_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    // commands
    rclcpp::Publisher<communication::msg::ActuatorCmds>::SharedPtr
        actuators_cmds_pub_ptr_;
    // obs
    rclcpp::Subscription<communication::msg::MotionCommands>::SharedPtr 
        motion_commands_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<communication::msg::TouchSensor>::SharedPtr
        touch_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joints_state_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    Client<communication::srv::RobotReset>::SharedPtr reset_state_client_;
    Client<communication::srv::SimulationReset>::SharedPtr simulation_reset_client_;
    // publisher for logs
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;
};
