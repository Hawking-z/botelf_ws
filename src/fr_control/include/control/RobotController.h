
#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <communication/msg/touch_sensor.hpp>
#include <communication/srv/robot_reset.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <communication/srv/simulation_reset.hpp>

// #include "debug_msgs/msg/observation.hpp"
// #include "debug_msgs/msg/action.hpp"

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
#include <yaml-cpp/yaml.h>

using namespace rclcpp;
using namespace std::chrono_literals;

class RobotController : public Node
{
public:
    RobotController();
    ~RobotController();

    void init();

private:
    void inner_loop();
    void load_conifg(std::string config_path);
    void reset_robot(int reset_step, bool release);
    void init_robot_pos();
    void update_obs();
    void update_run_obs();
    void infer_action();
    void infer_run_action();
    void ov_test_infer_action();

    double smooth_sqr_wave(double sin_phase,double eps = 0.25);

    void commands_callback(const communication::msg::MotionCommands::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void touch_callback(const communication::msg::TouchSensor::SharedPtr msg);
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void observation_callback(const communication::msg::TouchSensor::SharedPtr msg);
    void action_callback(const communication::msg::TouchSensor::SharedPtr msg);
    void callSimulationResetService();

    Eigen::VectorXf reorderVector(const Eigen::VectorXf &input, const std::vector<int> &mapping);

private:
    std::string topic_prefix_;

    rclcpp::Publisher<communication::msg::ActuatorCmds>::SharedPtr
        actuators_cmds_pub_ptr_;
    rclcpp::Publisher<communication::msg::TouchSensor>::SharedPtr
        observation_pub_ptr_;
    rclcpp::Publisher<communication::msg::TouchSensor>::SharedPtr
        action_pub_ptr_;

    rclcpp::Subscription<communication::msg::TouchSensor>::SharedPtr
        observation_subscription_;
    rclcpp::Subscription<communication::msg::TouchSensor>::SharedPtr
        action_subscription_;

    rclcpp::Subscription<communication::msg::MotionCommands>::SharedPtr 
        motion_commands_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<communication::msg::TouchSensor>::SharedPtr
        touch_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joints_state_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    std::thread inner_loop_thread_;

    Client<communication::srv::RobotReset>::SharedPtr reset_state_client_;
    Client<communication::srv::SimulationReset>::SharedPtr client_;

    bool run_ = false;

    bool error_flag_ = false;
    int error_count = 0;
    bool pd_controller_flag_ = false;

    const double dt_ = 0.001;     //pd频率
    rclcpp::Rate loop_rate_;
    size_t loop_count_ = 0;

    // --------通过yaml初始化--------
    float pd_rate_;   //pd频率
    float infer_rate_;   
    int decimation_ = 10;    //rl频率降低比例
    int obs_dim_ = 86;       //观测维度
    int stack_size_ = 15;     //观测堆叠
    int used_size_ = 5;
    int action_dim_ = 20;    //动作维度
    int joint_num_ = 20;     //关节数量
    float cycle_time_ = 0.6; //周期时间
    size_t max_time_;
    bool use_res_ = true;
    bool is_keyboard_ = false;
    std::vector<std::string> joint_names_;
    YAML::Node config_;
    
    //Mutexes
    std::mutex motion_commands_mutex_;
    std::mutex imu_mutex_;
    std::mutex joint_mutex_;
    std::mutex touch_mutex_;
    std::mutex odom_mutex_;

    //Sensor data & Commands
    communication::msg::MotionCommands::SharedPtr motion_commands_msg_;
    sensor_msgs::msg::Imu::SharedPtr imu_msg_;
    sensor_msgs::msg::JointState::SharedPtr joint_state_msg_;
    communication::msg::TouchSensor::SharedPtr touch_sensor_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    //observation, action
    communication::msg::TouchSensor::SharedPtr observation_msg_;
    communication::msg::TouchSensor::SharedPtr action_msg_;
    std::mutex observation_mutex_;
    std::mutex action_mutex_;

    // Eigen::Vector3f imu_ang_vel_;
    // Eigen::Vector3f imu_lin_acc_;
    // Eigen::VectorXf imu_quat_xyzw_;
    Eigen::VectorXf joint_pos_;
    Eigen::VectorXf joint_vel_;
    Eigen::Vector3d imu_ang_vel_;
    Eigen::Quaterniond root_quat_;
    // Eigen::VectorXf joint_torque_;

    //OpenVINO Model
    std::string ov_model_path_origin_;
    ov::Core ov_core_origin_;

    ov::element::Type ov_input_type_;
    ov::Shape ov_input_shape_;
    ov::CompiledModel ov_compiled_model_;
    ov::InferRequest ov_infer_request_;

    // ov::Model ov_model_;
    ov::CompiledModel ov_compiled_model_origin_;
    ov::InferRequest ov_infer_request_origin_;

    std::string ov_model_path_res_;
    ov::Core ov_core_res_;
    ov::CompiledModel ov_compiled_model_res_;
    ov::InferRequest ov_infer_request_res_;

    //Model input/output data
    Eigen::VectorXf obs_cur_; // 86
    std::deque<Eigen::VectorXf> obs_buffer_;
    Eigen::VectorXf obs_input_; // 86*5+1
    Eigen::VectorXf res_input_; // 86*5+1+20
    Eigen::VectorXf action_output_; // 20
    Eigen::VectorXf action_output_prev_; // 12
    Eigen::VectorXf target_torques_;
    Eigen::VectorXf target_torques_prev_;
    
    // clock signal
    double total_time_ = 0;
    double sin_phase_ = 0;
    double cos_phase_ = 0;
    double contact_schedule_ = 0;

    double eps_ = 0.2;

    //Controller parameters
    Eigen::VectorXf joint_kp_;
    Eigen::VectorXf joint_kd_;
    Eigen::VectorXf default_dof_pos_;
    Eigen::VectorXf target_dof_pos_;
    Eigen::VectorXf init_base_;
    Eigen::VectorXf init_kp_;
    std::unordered_map<std::string, float> obs_scales_;
    float action_scales_;
    float clip_observations_; 
    float clip_actions_;
    Eigen::VectorXf torque_limits_;

    Eigen::VectorXf torque_limits_low_;
    Eigen::VectorXf torque_limits_high_;

    // 网络和节点关节顺序不同，需要重排列
    std::vector<int> nn_to_node_mapping_;
    std::vector<int> node_to_nn_mapping_;
};
