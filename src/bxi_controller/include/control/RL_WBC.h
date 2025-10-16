
#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <communication/msg/touch_sensor.hpp>
#include <communication/srv/robot_reset.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <communication/msg/actuator_cmds.hpp>
#include <communication/msg/motion_commands.hpp>

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <eigen3/Eigen/Dense>
#include <openvino/openvino.hpp>

#include <deque>
#include <unordered_map>
#include <cmath>
#include <thread>
#include <chrono>

#include "utils/math_tools.h"

#include <std_msgs/msg/float32_multi_array.hpp>

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
    void init_robot_pos();
    void update_obs();
    void infer_action();

    void load_config(std::string config_path);

    void infer_loop();
    void pd_controller_loop();
    void main_thread();
 
    void commands_callback(const communication::msg::MotionCommands::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void touch_callback(const communication::msg::TouchSensor::SharedPtr msg);
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    double smooth_sqr_wave(double sin_phase,double eps = 0.25);

private:
    std::string topic_prefix_;

    rclcpp::Publisher<communication::msg::ActuatorCmds>::SharedPtr
        actuators_cmds_pub_ptr_;

    rclcpp::Subscription<communication::msg::MotionCommands>::SharedPtr 
        motion_commands_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<communication::msg::TouchSensor>::SharedPtr
        touch_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joints_state_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    Client<communication::srv::RobotReset>::SharedPtr reset_state_client_;

    // publisher for logs
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;

    Eigen::VectorXd joint_pos_;
    Eigen::VectorXd joint_vel_;
    Eigen::Vector3d imu_ang_vel_;
    Eigen::Quaterniond root_quat_;


    std::thread infer_loop_thread_;
    std::thread pd_controller_thread_;
    std::thread main_thread_;

    //Mutexes
    std::mutex motion_commands_mutex_;
    std::mutex imu_mutex_;
    std::mutex joint_mutex_;
    std::mutex touch_mutex_;
    std::mutex odom_mutex_;

    bool run_ = false;

    bool error_flag_ = false;
    int error_count = 0;
    bool pd_controller_flag_ = false;

    // --------通过yaml初始化--------
    float pd_rate_;   //pd频率
    float infer_rate_;   
    
    int obs_dim_;       //观测维度
    int stack_size_;     //观测堆叠
    int action_dim_;    //动作维度
    int joint_num_;     //关节数量
    float cycle_time_; //周期时间
   
    Eigen::VectorXf joint_kp_;
    Eigen::VectorXf joint_kd_;
    Eigen::VectorXf default_dof_pos_;
    Eigen::VectorXf init_dof_pos_;
    std::unordered_map<std::string, float> obs_scales_;

    float action_scales_;
    float clip_observations_; 
    float clip_actions_;
    Eigen::VectorXf torque_limits_;
    std::vector<std::string> joint_names_;

    // lowPassFilter
    // LowPassFilter<Eigen::VectorXf> lowpass_filter_1_;

    //OpenVINO Model
    ov::element::Type ov_input_type_;
    ov::Shape ov_input_shape_;
    ov::CompiledModel ov_compiled_model_;
    ov::InferRequest ov_infer_request_;

    //Model input/output data
    Eigen::VectorXf obs_cur_; // 47
    std::deque<Eigen::VectorXf> obs_buffer_;
    Eigen::VectorXf obs_input_; // 47*3
    Eigen::VectorXf action_output_; // 12
    Eigen::VectorXf action_output_prev_; // 12
    Eigen::VectorXf target_torques_;
    Eigen::VectorXf target_torques_prev_;

    // clock signal
    double total_time_ = 0;
    double sin_phase_ = 0;
    double cos_phase_ = 0;
    double contact_schedule_ = 0;

    double eps_ = 0.2;
    
    //Sensor data & Commands
    communication::msg::MotionCommands::SharedPtr motion_commands_msg_;
    sensor_msgs::msg::Imu::SharedPtr imu_msg_;
    sensor_msgs::msg::JointState::SharedPtr joint_state_msg_;
    communication::msg::TouchSensor::SharedPtr touch_sensor_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;
};
