/*
 * @Author: mashuaikang 
 * @Date: 2024-07-21 23:48:19 
 * @Last Modified by:   mashuaikang 
 * @Last Modified time: 2024-07-21 23:48:19 
 */

#define PUBLISH_RL_MSG
// #define OV_TEST

#include "control/RobotController.h"
#include <rcpputils/asserts.hpp>

float adjust_rpy(float angle)
{
    if (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    else if (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

RobotController::RobotController() : Node("RobotController"), loop_rate_(1.0 / dt_)
{
    this->declare_parameter("/topic_prefix", "");
    
    topic_prefix_ = this->get_parameter("/topic_prefix")
                        .get_parameter_value()
                        .get<std::string>();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto node_info_list = this->get_node_graph_interface()->get_node_names_and_namespaces();
    // 遍历节点列表，看是否有以 "keyboard_controller" 或 "remote_controller" 开头的名字
    for (const auto & [name, ns] : node_info_list) {
      if (name.rfind("keyboard_controller", 0) == 0) {
        is_keyboard_ = true;
      }
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    actuators_cmds_pub_ptr_ =
        this->create_publisher<communication::msg::ActuatorCmds>(
            topic_prefix_ + "actuators_cmds", qos);

        observation_pub_ptr_ =
        this->create_publisher<communication::msg::TouchSensor>(
            topic_prefix_ + "observation", qos);
    action_pub_ptr_ =
        this->create_publisher<communication::msg::TouchSensor>(
            topic_prefix_ + "action", qos);
#ifdef OV_TEST
    observation_subscription_ = 
        this->create_subscription<communication::msg::TouchSensor>(
            "/observation_py", qos,
            std::bind(&RobotController::observation_callback, this,
                      std::placeholders::_1));
    action_subscription_ = 
        this->create_subscription<communication::msg::TouchSensor>(
            "/action_py", qos,
            std::bind(&RobotController::action_callback, this,
                      std::placeholders::_1));
#endif

    joints_state_subscription_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            topic_prefix_ + "joint_states", qos,
            std::bind(&RobotController::joint_callback, this,
                      std::placeholders::_1));
    
    motion_commands_subscription_ =
        this->create_subscription<communication::msg::MotionCommands>(
            "/motion_commands", qos,
            std::bind(&RobotController::commands_callback, this,
                      std::placeholders::_1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        topic_prefix_ + "imu_data", qos,
        std::bind(&RobotController::imu_callback, this, std::placeholders::_1));

    odom_subscription_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            topic_prefix_ + "odom", qos,
            std::bind(&RobotController::odom_callback, this,
                      std::placeholders::_1));

    touch_subscription_ =
        this->create_subscription<communication::msg::TouchSensor>(
            topic_prefix_ + "touch_sensor", qos,
            std::bind(&RobotController::touch_callback, this,
                      std::placeholders::_1));

    reset_state_client_ =
        this->create_client<communication::srv::RobotReset>(
            topic_prefix_ + "robot_reset");

    client_ =
        this->create_client<communication::srv::SimulationReset>(
            topic_prefix_ + "sim_reset");

    //Sensor data
    // imu_quat_xyzw_.resize(4);
    joint_pos_.resize(joint_num_);
    joint_vel_.resize(joint_num_);
    // joint_torque_.resize(joint_num_);

    config_ = YAML::LoadFile("src/fr_control/config.json");

    max_time_ = config_["max_time"].as<size_t>();

    use_res_ = config_["use_res"].as<bool>();

    //OpenVINO Model
    ov_model_path_origin_ = config_["net_path"]["origin"].as<std::string>();
    // auto ov_model = ov_core_.read_model(ov_model_path_);
    // ov_compiled_model_ = ov_core_.compile_model(ov_model, "AUTO:CPU,GPU.0", 
    //     ov::hint::performance_mode(ov::hint::PerformanceMode::CUMULATIVE_THROUGHPUT));
    ov_compiled_model_origin_ = ov_core_origin_.compile_model(ov_model_path_origin_, "CPU");
    ov_infer_request_origin_ = ov_compiled_model_origin_.create_infer_request();

    if(use_res_)
    {
        ov_model_path_res_ = config_["net_path"]["res"].as<std::string>();
        ov_compiled_model_res_= ov_core_res_.compile_model(ov_model_path_res_, "CPU");
        ov_infer_request_res_ = ov_compiled_model_res_.create_infer_request();
    }

    //Model input/output data
    obs_cur_.resize(obs_dim_); // 86*15
    obs_cur_.setZero();
    for(int i = 0; i < stack_size_; i++)
    {
        obs_buffer_.push_back(obs_cur_);
    }
    obs_input_.resize(obs_dim_*used_size_ + 1); // 86*5+1
    res_input_.resize(obs_dim_*used_size_ + 1 + action_dim_);
    action_output_.resize(action_dim_); // 20
    action_output_.setZero();
    target_torques_.resize(action_dim_);
    target_torques_.setZero();
    target_torques_prev_.resize(action_dim_);
    target_torques_prev_.setZero();

    //Controller parameters
    joint_kp_.resize(joint_num_);
    joint_kd_.resize(joint_num_);
    joint_kp_ << 25, 25, 30, 40, 3, 3,
                 25, 25, 30, 40, 3, 3,
                 25, 25, 3, 25,
                 25, 25, 3, 25;
    // joint_kp_ <<    30,30,38,48,4,4,
    //                 30,30,38,48,4,4,
    //                 5, 5, 5, 5,
    //                 5, 5, 5, 5;
    joint_kd_ << 2.5, 2.5, 3.0, 4.0, 0.3, 0.3,
                 2.5, 2.5, 3.0, 4.0, 0.3, 0.3,
                 2.5, 2.5, 0.3, 2.5,
                 2.5, 2.5, 0.3, 2.5;

    default_dof_pos_.resize(joint_num_);
    std::vector<float> default_dof_pos_std = config_["default_positions"].as<std::vector<float>>();
    default_dof_pos_ = Eigen::VectorXf::Map(default_dof_pos_std.data(), default_dof_pos_std.size());

    target_dof_pos_.resize(joint_num_);
    std::vector<float> target_dof_pos_std = config_["target_positions"].as<std::vector<float>>();
    target_dof_pos_ = Eigen::VectorXf::Map(target_dof_pos_std.data(), target_dof_pos_std.size());

    init_base_.resize(7);
    std::vector<float> init_base_std = config_["init_bases"].as<std::vector<float>>();
    init_base_ = Eigen::VectorXf::Map(init_base_std.data(), init_base_std.size());

    init_kp_.resize(joint_num_);
    std::vector<float> init_kp_std = config_["init_kp"].as<std::vector<float>>();
    init_kp_ = Eigen::VectorXf::Map(init_kp_std.data(), init_kp_std.size());

    obs_scales_["lin_vel"] = 1.0;
    obs_scales_["ang_vel"] = 1.0;
    obs_scales_["dof_pos"] = 1.0;
    obs_scales_["dof_vel"] = 0.05;
    obs_scales_["quat"] = 1.0;
    obs_scales_["height_measurements"] = 5.0;
    action_scales_ = 1.0;
    clip_observations_ = 18.; 
    clip_actions_ = 18.;
    
    torque_limits_.resize(joint_num_);
    torque_limits_low_.resize(joint_num_);
    torque_limits_high_.resize(joint_num_);
    torque_limits_ << 14.25, 30.015, 40, 60, 7.2, 7.2,
                      14.25, 30.015, 40, 60, 7.2, 7.2,
                      15, 15, 5.6, 15,
                      15, 15, 5.6, 15;
    torque_limits_low_ << -5, -5, -10, -20, -3, -3,
                          -5, -5, -10, -20, -3, -3,
                          -5, -5, -5, -5,
                          -5, -5, -5, -5;
    torque_limits_high_ << 5, 5, 10, 10, 3, 3,
                           5, 5, 10, 10, 3, 3,
                           5, 5, 5, 5,
                           5, 5, 5, 5;

    nn_to_node_mapping_ = {0, 1, 2, 3, 4, 5,
                           10, 11, 12, 13, 14, 15,
                           6, 7, 8, 9,
                           16, 17, 18, 19};
    node_to_nn_mapping_ = {0, 1, 2, 3, 4, 5,
                           12, 13, 14, 15,
                           6, 7, 8, 9, 10, 11,
                           16, 17, 18, 19};

    joint_names_ = {
        "l_hip_z_joint",
        "l_hip_x_joint",
        "l_hip_y_joint",
        "l_knee_y_joint",
        "l_ankle_y_joint",
        "l_ankle_x_joint",

        "r_hip_z_joint",
        "r_hip_x_joint",
        "r_hip_y_joint",
        "r_knee_y_joint",
        "r_ankle_y_joint",
        "r_ankle_x_joint",

        "l_shld_y_joint",
        "l_shld_x_joint",
        "l_shld_z_joint",
        "l_elb_y_joint",

        "r_shld_y_joint",
        "r_shld_x_joint",
        "r_shld_z_joint",
        "r_elb_y_joint"};

    RCLCPP_INFO(this->get_logger(), "RobotController initialized");
}

void RobotController::reset_robot(int reset_step, bool release)
{

    auto request =
        std::make_shared<communication::srv::RobotReset::Request>();

    request->header.frame_id = std::string("bot_elf");
    request->reset_step = reset_step;
    request->release = release;

    RCLCPP_INFO(this->get_logger(), "waiting for service %s ...",
                reset_state_client_->get_service_name());
    while (!reset_state_client_->wait_for_service(20ms))
    {
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service %s ...",
                reset_state_client_->get_service_name());
    }

    auto result = reset_state_client_->async_send_request(request);

    if (result.get()->is_success)
    {
        RCLCPP_INFO(this->get_logger(), "call service reset_state success");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to reset state");
    }
}

void RobotController::commands_callback(
    const communication::msg::MotionCommands::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(motion_commands_mutex_);
    motion_commands_msg_ = msg;
}

void RobotController::imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_msg_ = msg;
}

void RobotController::joint_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_mutex_);
    joint_state_msg_ = msg;
    // 将 joint_state_msg_ 数据转换为 float 并赋值到 joint_pos_ 和 joint_vel_
    for (int i = 0; i < joint_num_; ++i) {
        joint_pos_(i) = static_cast<float>(joint_state_msg_->position[i]);
        joint_vel_(i) = static_cast<float>(joint_state_msg_->velocity[i]);
    }
}

void RobotController::touch_callback(
    const communication::msg::TouchSensor::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(touch_mutex_);
    touch_sensor_msg_ = msg;
}

//only for simulation
void RobotController::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_msg_ = msg;
}

void RobotController::observation_callback(
    const communication::msg::TouchSensor::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(observation_mutex_);
    observation_msg_ = msg;
}

void RobotController::action_callback(
    const communication::msg::TouchSensor::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(action_mutex_);
    action_msg_ = msg;
}


void RobotController::init()
{
    inner_loop_thread_ = std::thread(&RobotController::inner_loop, this);
    run_ = true;
}

Eigen::VectorXf RobotController::reorderVector(const Eigen::VectorXf& input, const std::vector<int>& mapping) 
{
    Eigen::VectorXf output(mapping.size());
    for (size_t i = 0; i < mapping.size(); ++i) 
    {
        output[i] = input[mapping[i]];
    }
    return output;
}

RobotController::~RobotController()
{
    run_ = false;
    inner_loop_thread_.join();
}

void RobotController::callSimulationResetService()
{
    // 等待 service 可用
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "等待 sim_reset 服务上线...");
    }

    // 构造请求
    auto request = std::make_shared<communication::srv::SimulationReset::Request>();

    request->header.frame_id = std::string("bot_elf");

    // 示例：给 base_pose 赋值
    request->base_pose.position.x = init_base_[0];
    request->base_pose.position.y = init_base_[1];
    request->base_pose.position.z = init_base_[2] + 0.9;
    request->base_pose.orientation.x = init_base_[3];
    request->base_pose.orientation.y = init_base_[4];
    request->base_pose.orientation.z = init_base_[5];
    request->base_pose.orientation.w = init_base_[6];

    // 示例：根据需要设置关节信息
    for (int i = 0; i < joint_num_; i++)
    {
        request->joint_state.name.emplace_back(joint_names_[i]);
        request->joint_state.position.emplace_back(default_dof_pos_(i));
        request->joint_state.velocity.emplace_back(0);
        request->joint_state.effort.emplace_back(0);
    }

    // 以异步方式发送请求
    using ServiceResponseFuture =
      rclcpp::Client<communication::srv::SimulationReset>::SharedFuture;
    auto response_received_callback =
      [this](ServiceResponseFuture future) -> void
    {
      // 处理服务器端返回结果
      auto response = future.get();
      if (response->is_success) 
      {
        RCLCPP_INFO(this->get_logger(), "SimulationReset success: is_success=true");
      } else {
        RCLCPP_WARN(this->get_logger(), "SimulationReset false: is_success=false");
      }
    };

    // 调用异步请求
    auto future_result = client_->async_send_request(request, response_received_callback);
}

//根据初始化的default_dof_pos_对关节位置进行简单的pd控制
void RobotController::init_robot_pos()
{
    RCLCPP_INFO(this->get_logger(), "Robot Joint_Pos Init start!");
    size_t loop_count = 0;
    double soft_start = 0;
    // init motor pos
    while (rclcpp::ok() && run_)
    {
        communication::msg::ActuatorCmds actuatorCmds;
        actuatorCmds.header.frame_id = std::string("bot_elf");
        actuatorCmds.header.stamp = this->now();
        // update_obs();

        for (int i = 0; i < joint_num_; i++)
        {
            actuatorCmds.actuators_name.emplace_back(joint_names_[i]);
            actuatorCmds.pos.emplace_back(default_dof_pos_(i));
            actuatorCmds.vel.emplace_back(0);
            actuatorCmds.torque.emplace_back(0);

            soft_start = loop_count / (1. / dt_);
            soft_start = soft_start > 1 ? 1 : soft_start;

            actuatorCmds.kp.emplace_back(init_kp_[i] * soft_start);
            actuatorCmds.kd.emplace_back(init_kp_[i] * 0.1);
        }

        actuators_cmds_pub_ptr_->publish(actuatorCmds);

        loop_count++;
        {
            std::lock_guard<std::mutex> lock(motion_commands_mutex_);
            if (is_keyboard_){
                if (loop_count > (3. / dt_) && (motion_commands_msg_->vel_des.x>0)) //hold for 3s
                    break;
            }
            else {
                if (loop_count > (3. / dt_) && ((motion_commands_msg_->mode)%2==0)) //hold for 3s
                    break;
            }
        }//限制lock_guard作用域
        loop_rate_.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Robot Joint_Pos Init done!");
}

//根据传感器数据和指令数据更新观测数据obs_buffer
void RobotController::update_obs()
{
// #ifdef OV_TEST
//     std::lock_guard<std::mutex> lock(observation_mutex_);
//     obs_cur_.setZero();
//     for(int i=0; i<obs_dim_; i++)
//     {
//         obs_cur_(i) = observation_msg_->value[i];
//     }
//     obs_buffer_.pop_front();
//     obs_buffer_.push_back(obs_cur_);


//     communication::msg::TouchSensor obs_msg_p;
//     obs_msg_p.header.frame_id = std::string("bot_elf");
//     obs_msg_p.header.stamp = this->now();
//     for(int i=0; i<obs_dim_; i++)
//     {
//         RCLCPP_INFO(this->get_logger(), "obs_cur_%i:%f", i,  obs_cur_(i));
//         obs_msg_p.value.emplace_back(obs_cur_(i));
//     }
//     observation_pub_ptr_->publish(obs_msg_p);


//     return;
// #endif

    std::lock_guard<std::mutex> cmd_lock(motion_commands_mutex_);
    std::lock_guard<std::mutex> imu_lock(imu_mutex_);
    std::lock_guard<std::mutex> joint_lock(joint_mutex_);
    // std::lock_guard<std::mutex> touch_lock(touch_mutex_);
    // std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    //clock input
    //joint input
    // for (int i = 0; i < 12; ++i) {
    //     obs_cur_(5 + i) = static_cast<float>(joint_state_msg_->position[i] * obs_scales_["dof_pos"]);
    //     obs_cur_(17 + i) = static_cast<float>(joint_state_msg_->velocity[i] * obs_scales_["dof_vel"]);
    // }
    obs_cur_.segment(0, 20) = reorderVector(joint_pos_ - default_dof_pos_, node_to_nn_mapping_) * obs_scales_["dof_pos"];
    obs_cur_.segment(20, 20) = reorderVector(joint_vel_, node_to_nn_mapping_) * obs_scales_["dof_vel"];
    obs_cur_.segment(40, 20) = reorderVector(joint_pos_ - target_dof_pos_, node_to_nn_mapping_) * obs_scales_["dof_pos"] ;
    //last action
    obs_cur_.segment(60, 20) = action_output_;
    //imu input
    obs_cur_(80) = static_cast<float>(imu_msg_->angular_velocity.x * obs_scales_["ang_vel"]);
    obs_cur_(81) = static_cast<float>(imu_msg_->angular_velocity.y * obs_scales_["ang_vel"]);
    obs_cur_(82) = static_cast<float>(imu_msg_->angular_velocity.z * obs_scales_["ang_vel"]);

    // Eigen::Quaternionf q(
    //     static_cast<float>(imu_msg_->orientation.w),
    //     static_cast<float>(imu_msg_->orientation.x),
    //     static_cast<float>(imu_msg_->orientation.y),
    //     static_cast<float>(imu_msg_->orientation.z)
    // );
    // Eigen::Vector3f euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2); // 用于提取 roll, pitch, yaw
    float x = imu_msg_->orientation.x;
    float y = imu_msg_->orientation.y;
    float z = imu_msg_->orientation.z;
    float w = imu_msg_->orientation.w;

    float t0 = +2.0f * (w * x + y * z);
    float t1 = +1.0f - 2.0f * (x * x + y * y);
    obs_cur_(83) = std::atan2(t0, t1) * obs_scales_["quat"];

    float t2 = +2.0f * (w * y - z * x);
    t2 = ((t2 > 1.0f) ? 1.0f : t2);
    t2 = ((t2 < -1.0f) ? -1.0f : t2);
    obs_cur_(84) = std::asin(t2) * obs_scales_["quat"];

    float t3 = +2.0f * (w * z + x * y);
    float t4 = +1.0f - 2.0f * (y * y + z * z);
    obs_cur_(85) = std::atan2(t3, t4) * obs_scales_["quat"];

    // obs_cur_(44) = adjust_rpy(euler_angles(0)) * obs_scales_["quat"];
    // obs_cur_(45) = adjust_rpy(euler_angles(1)) * obs_scales_["quat"];
    // obs_cur_(46) = adjust_rpy(euler_angles(2)) * obs_scales_["quat"];

    obs_buffer_.pop_front();
    obs_buffer_.push_back(obs_cur_);

#ifdef PUBLISH_RL_MSG
    communication::msg::TouchSensor obs_msg;
    obs_msg.header.frame_id = std::string("bot_elf");
    obs_msg.header.stamp = this->now();
    for(int i=0; i<obs_dim_; i++)
    {
        // RCLCPP_INFO(this->get_logger(), "obs_cur_%i:%f", i,  obs_cur_(i));
        obs_msg.value.emplace_back(obs_cur_(i));
    }
    observation_pub_ptr_->publish(obs_msg);
#endif

}

void RobotController::update_run_obs()
{
    try
    {   
      
        obs_cur_.setZero();
        
        {
            std::lock_guard<std::mutex> lock1(imu_mutex_);
            std::lock_guard<std::mutex> lock2(joint_mutex_);
            std::lock_guard<std::mutex> lock3(motion_commands_mutex_);

            imu_ang_vel_(0) = imu_msg_->angular_velocity.x;
            imu_ang_vel_(1) = imu_msg_->angular_velocity.y;
            imu_ang_vel_(2) = imu_msg_->angular_velocity.z;

            root_quat_.x() = imu_msg_->orientation.x;
            root_quat_.y() = imu_msg_->orientation.y;
            root_quat_.z() = imu_msg_->orientation.z;
            root_quat_.w() = imu_msg_->orientation.w;

            for (int i = 0; i < joint_num_; ++i)
            {   
                joint_pos_(i) = static_cast<double>(joint_state_msg_->position[i]);
                joint_vel_(i) = static_cast<double>(joint_state_msg_->velocity[i]);
            }
            obs_cur_(6) = motion_commands_msg_->vel_des.x * obs_scales_["lin_vel"];
            obs_cur_(7) = motion_commands_msg_->vel_des.y * obs_scales_["lin_vel"]; 
            obs_cur_(8) = motion_commands_msg_->yawdot_des * obs_scales_["ang_vel"];
        }

        Eigen::Vector3d root_euler = get_euler_xyz(root_quat_);
        Eigen::VectorXf joint_pos = (joint_pos_.cast<float>().segment(0, 12) - default_dof_pos_.head(12))*obs_scales_["dof_pos"];
        Eigen::VectorXf joint_vel = joint_vel_.cast<float>().segment(0, 12) * obs_scales_["dof_vel"];
  
        if (pd_controller_flag_)
        {
            if(std::fabs(root_euler(0)) > 0.8 || std::fabs(root_euler(1)) > 0.8)
            {
                error_count += 1;
            }else
            {
                error_count = 0;
            }
            if(error_count > 10)
            {
                // RCLCPP_ERROR(this->get_logger(), "RobotController: Robot fall down!");
                error_flag_ = true;
            }
        }
        
        obs_cur_(0) = imu_ang_vel_(0) * obs_scales_["ang_vel"];
        obs_cur_(1) = imu_ang_vel_(1) * obs_scales_["ang_vel"];
        obs_cur_(2) = imu_ang_vel_(2) * obs_scales_["ang_vel"];

        obs_cur_(3) = root_euler[0] * obs_scales_["quat"];
        obs_cur_(4) = root_euler[1] * obs_scales_["quat"];
        obs_cur_(5) = root_euler[2] * obs_scales_["quat"];

        obs_cur_(9) = sin_phase_;
        obs_cur_(10) = cos_phase_;
        obs_cur_(11) = contact_schedule_;
        
        obs_cur_.segment(12, 12) = joint_pos;
        obs_cur_.segment(24, 12) = joint_vel;
        obs_cur_.segment(36, 12) =  action_output_;
        
        // std::cout<<obs_cur_.transpose()<<std::endl;
        obs_buffer_.pop_front();
        obs_buffer_.push_back(obs_cur_);
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(this->get_logger(), "update_obs error");
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void RobotController::ov_test_infer_action()
{
    #include <chrono>
    size_t count = 0;
    auto start = std::chrono::high_resolution_clock::now();
    while(true){
        Eigen::VectorXd obs_input_(431);
        obs_input_ = Eigen::VectorXd::Random(431);
        auto ov_input_port = ov_compiled_model_origin_.input();
        // Create tensor from external memory
        ov::Tensor ov_input_tensor(ov_input_port.get_element_type(), ov_input_port.get_shape(), obs_input_.data());
        // Set input tensor for model with one input
        ov_infer_request_origin_.set_input_tensor(ov_input_tensor);
        ov_infer_request_origin_.start_async();
        ov_infer_request_origin_.wait();
        // Get output tensor & set action_output_
        const ov::Tensor& output_tensor = ov_infer_request_origin_.get_output_tensor();
        auto output_buf = output_tensor.data<const float>();
        for (int i = 0; i < action_dim_; ++i) {
            action_output_(i) = output_buf[i];
        }
        action_output_ = action_output_.cwiseMin(clip_actions_).cwiseMax(-clip_actions_);

        count++;
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        // std::cout << "Elapsed time: " << duration.count()/count << " microseconds" << std::endl;
        RCLCPP_INFO(this->get_logger(), "Elapsed time: %e",  duration.count()*1.0/count);

    }  
}
void RobotController::infer_run_action()
{
    try
    {

        // std::cout<<"infer_run_action"<<std::endl;
        size_t data_index = 0;

        for (const auto &data : obs_buffer_)
        {
            obs_input_.segment(data_index, data.size()) = data;
            data_index += data.size();
        }
        obs_input_ = obs_input_.cwiseMin(clip_observations_).cwiseMax(-clip_observations_);
        auto ov_input_port = ov_compiled_model_.input();
        // Create tensor from external memory
        ov::Tensor ov_input_tensor(ov_input_type_, ov_input_shape_, obs_input_.data());

        ov_infer_request_.set_input_tensor(ov_input_tensor);
        ov_infer_request_.start_async();
        ov_infer_request_.wait();
        // Get output tensor & set action_output_
        const ov::Tensor &output_tensor = ov_infer_request_.get_output_tensor();
        auto output_buf = output_tensor.data<const float>();
        for (int i = 0; i < action_dim_; ++i)
        {
            action_output_(i) = output_buf[i];
        }
        action_output_ = action_output_.cwiseMin(clip_actions_).cwiseMax(-clip_actions_);

    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(this->get_logger(), "infer_action error");
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void RobotController::infer_action()
{
    std::vector<int> indices;
    indices.push_back(0);
    int last_index = stack_size_ - 1;
    float interval = last_index / static_cast<float>(used_size_ - 1);
    for (int i = 1; i < used_size_ - 1; ++i)
    {
        indices.push_back(static_cast<int>(i * interval));
    }
    indices.push_back(last_index);

    obs_input_(0) = loop_count_ * dt_;
    // printf("time: %f\n", loop_count_ * dt_);
    for (int i = 0; i < used_size_; ++i) 
    {
        obs_input_.segment(i * obs_dim_ + 1, obs_dim_) = obs_buffer_[indices[i]];
    }
    obs_input_ = obs_input_.cwiseMin(clip_observations_).cwiseMax(-clip_observations_);
    auto ov_input_port = ov_compiled_model_origin_.input();
    // Create tensor from external memory
    ov::Tensor ov_input_tensor(ov_input_port.get_element_type(), ov_input_port.get_shape(), obs_input_.data());
    // Set input tensor for model with one input
    ov_infer_request_origin_.set_input_tensor(ov_input_tensor);
    ov_infer_request_origin_.start_async();
    ov_infer_request_origin_.wait();
    // Get output tensor & set action_output_
    const ov::Tensor& output_tensor = ov_infer_request_origin_.get_output_tensor();
    auto output_buf = output_tensor.data<const float>();
    for (int i = 0; i < action_dim_; ++i) 
    {
        action_output_(i) = output_buf[i];
    }

    if(use_res_)
    {
        res_input_.segment(0, obs_dim_*used_size_ + 1) = obs_input_;
        res_input_.segment(obs_dim_*used_size_ + 1, action_dim_) = action_output_;
        res_input_ = res_input_.cwiseMin(clip_observations_).cwiseMax(-clip_observations_);
        auto ov_input_port = ov_compiled_model_res_.input();
        // Create tensor from external memory
        ov::Tensor ov_input_tensor(ov_input_port.get_element_type(), ov_input_port.get_shape(), res_input_.data());
        // Set input tensor for model with one input
        ov_infer_request_res_.set_input_tensor(ov_input_tensor);
        ov_infer_request_res_.start_async();
        ov_infer_request_res_.wait();
        // Get output tensor & set action_output_
        const ov::Tensor& output_tensor = ov_infer_request_res_.get_output_tensor();
        auto output_buf = output_tensor.data<const float>();
        for (int i = 0; i < action_dim_; ++i) 
        {
            action_output_(i) += 0.1 * output_buf[i];
        }
    }
    action_output_ = action_output_.cwiseMin(clip_actions_).cwiseMax(-clip_actions_);
    

#ifdef PUBLISH_RL_MSG
    
    communication::msg::TouchSensor action_msg;
    action_msg.header.frame_id = std::string("bot_elf");
    action_msg.header.stamp = this->now();
    for(int i=0; i<action_dim_; i++)
    {
        // RCLCPP_INFO(this->get_logger(), "action_output_%i:%f", i,  action_output_(i));
        action_msg.value.emplace_back(action_output_(i));
    }
    // action_msg.actions = std::vector<float>(action_output_.data(), action_output_.data() + action_output_.size());
    action_pub_ptr_->publish(action_msg);
#endif

}

void RobotController::load_config(std::string filename)
{
    std::cout << "load config file: " << filename << std::endl;
    YAML::Node config = YAML::LoadFile(filename);
    pd_rate_ = config["pd_rate"].as<double>();
    infer_rate_ = config["infer_rate"].as<float>();
    obs_dim_ = config["obs_dim"].as<int>();
    cycle_time_ = config["cycle_time"].as<float>();
    stack_size_ = config["stack_size"].as<int>();
    action_dim_ = config["action_dim"].as<int>();
    joint_num_ = config["joint_num"].as<int>();
    action_scales_ = config["action_scales"].as<float>();
    clip_observations_ = config["clip_observations"].as<float>();
    clip_actions_ = config["clip_actions"].as<float>();
    // eps_ = config["eps"].as<double>();

    joint_kd_.resize(joint_num_);
    joint_kp_.resize(joint_num_);
    default_dof_pos_.resize(joint_num_);

    torque_limits_.resize(joint_num_);

    obs_scales_["lin_vel"] = config["obs_scales"]["lin_vel"].as<float>();
    obs_scales_["ang_vel"] = config["obs_scales"]["ang_vel"].as<float>();
    obs_scales_["dof_pos"] = config["obs_scales"]["dof_pos"].as<float>();
    obs_scales_["dof_vel"] = config["obs_scales"]["dof_vel"].as<float>();
    obs_scales_["quat"] = config["obs_scales"]["quat"].as<float>();

    size_t idx = 0;
    for (const auto &row : config["joint_kp"])
    {
        for (const auto &value : row)
        {
            joint_kp_(idx++) = value.as<float>();
        }
    }
    idx = 0;
    for (const auto &row : config["joint_kd"])
    {
        for (const auto &value : row)
        {
            joint_kd_(idx++) = value.as<float>();
        }
    }
    idx = 0;
    for (const auto &row : config["default_dof_pos"])
    {
        for (const auto &value : row)
        {
            default_dof_pos_(idx++) = value.as<float>();
        }
    }
    
    idx = 0;
    for (const auto &row : config["torque_limits"])
    {
        for (const auto &value : row)
        {
            torque_limits_(idx++) = value.as<float>();
        }
    }
    joint_names_.clear();
    for (const auto &name : config["joint_names"])
    {
        joint_names_.push_back(name.as<std::string>());
    }

    std::string ov_model_path_ = config["model"].as<std::string>();
    std::cout << "load ov model: " << ov_model_path_ << std::endl;
    ov::Core ov_core_;
    ov_compiled_model_ = ov_core_.compile_model(ov_model_path_, "CPU");
    auto ov_input_port = ov_compiled_model_.input();
    ov_input_type_ = ov_input_port.get_element_type();
    ov_input_shape_ = ov_input_port.get_shape();
    ov_infer_request_ = ov_compiled_model_.create_infer_request();

    RCLCPP_INFO(this->get_logger(), "param init !");
}


void RobotController::inner_loop()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

#ifdef OV_TEST
    ov_test_infer_action();
#endif

    printf("inner loop begin\n");
    //Hardware
    if (is_keyboard_ == false){
        while(!joint_state_msg_ || !imu_msg_ || !motion_commands_msg_ || motion_commands_msg_->mode != 1){
            // RCLCPP_INFO(this->get_logger(), "waiting for sensor data and command_data...");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    printf("inner loop start\n");
    // Enable motor position control
    reset_robot(1, false);
    // init robot joint pos while hang up
    init_robot_pos();
    printf("init success\n");
    // release robt(only for simulation), enable motor position, velocity and torque control
    reset_robot(2, true);

    //Simulation init base and wait
    if (topic_prefix_.find("simulation") != std::string::npos){
        loop_count_=0;
        callSimulationResetService();
        while (rclcpp::ok() && run_ && (loop_count_ < 100))
        {
            // publish actuatorCmds
            communication::msg::ActuatorCmds actuatorCmds;
            actuatorCmds.header.frame_id = std::string("bot_elf");
            actuatorCmds.header.stamp = this->now();
            {
                for (int i = 0; i < action_dim_; i++)
                {
                    actuatorCmds.actuators_name.emplace_back(joint_names_[i]);
                    actuatorCmds.pos.emplace_back(default_dof_pos_(i));
                    actuatorCmds.vel.emplace_back(-joint_vel_(i));
                    actuatorCmds.torque.emplace_back(0);

                    actuatorCmds.kp.emplace_back(init_kp_[i]);
                    actuatorCmds.kd.emplace_back(init_kp_[i] * 0.1);
                }
                actuators_cmds_pub_ptr_->publish(actuatorCmds);
            } // 限制lock_guard作用域
            // printf("loop_count_: %d\n", loop_count_);
            loop_count_++;
            loop_rate_.sleep();
        }
    }

    loop_count_ = 0;
    // run your control algorithm
    while (rclcpp::ok() && run_)
    {
        loop_count_ = (loop_count_ > max_time_) ? max_time_ : loop_count_;
        if (loop_count_ >= max_time_)
        {
            RCLCPP_INFO(this->get_logger(), "max_time_ reached, stop control");
            break;
        }
        if(loop_count_ % decimation_ == 0){
            //update obs
            update_obs(); //obs_buffer_ is updated
            //infer action
            infer_action(); //action_output_ is updated
        }

        //publish actuatorCmds
        communication::msg::ActuatorCmds actuatorCmds;
        actuatorCmds.header.frame_id = std::string("bot_elf");
        actuatorCmds.header.stamp = this->now();
        {
            Eigen::VectorXf action_for_node = reorderVector(action_output_, nn_to_node_mapping_);
            std::lock_guard<std::mutex> lock(joint_mutex_);
            // torque control
            target_torques_ = joint_kp_.cwiseProduct(
                        action_for_node * action_scales_ + default_dof_pos_ - joint_pos_) -
                        joint_kd_.cwiseProduct(joint_vel_);
            target_torques_ = target_torques_.cwiseMin(torque_limits_).cwiseMax(-torque_limits_);
            target_torques_ = 0.8 * target_torques_ + 0.2 * target_torques_prev_;
            target_torques_prev_ = target_torques_;

            //TODO butterworth filter
            
            for (int i = 0; i < action_dim_; i++)
            {
                actuatorCmds.actuators_name.emplace_back(joint_names_[i]);
                actuatorCmds.pos.emplace_back(0);
                actuatorCmds.vel.emplace_back(0);
                actuatorCmds.torque.emplace_back(target_torques_(i));

                actuatorCmds.kp.emplace_back(0);
                actuatorCmds.kd.emplace_back(0);
            }
            //position control
            // for (int i = 0; i < action_dim_; i++)
            // {
            //     actuatorCmds.actuators_name.emplace_back(joint_names_[i]);
            //     actuatorCmds.pos.emplace_back(action_output_(i)*action_scales_ +default_dof_pos_(i));
            //     actuatorCmds.vel.emplace_back(0);
            //     actuatorCmds.torque.emplace_back(0);

            //     actuatorCmds.kp.emplace_back(joint_kp_[i]);
            //     actuatorCmds.kd.emplace_back(joint_kd_[i]);
            // }

            actuators_cmds_pub_ptr_->publish(actuatorCmds);
        }//限制lock_guard作用域
        // printf("loop_count_: %d\n", loop_count_);
        loop_count_++;
        loop_rate_.sleep();
    }

    // run 
    loop_count_ = 0;
    load_config("src/bxi_controller/config/rl_controller.yaml");
    obs_cur_.resize(obs_dim_);
    obs_cur_.setZero();
    obs_buffer_.clear();
    for (int i = 0; i < stack_size_; i++)
    {
        obs_buffer_.push_back(obs_cur_);
    }
    obs_input_.resize(obs_dim_ * stack_size_);
    obs_input_.setZero();

    action_output_.resize(action_dim_);
    action_output_.setZero();
    action_output_prev_.resize(action_dim_);
    action_output_prev_.setZero();

    target_torques_.resize(action_dim_);
    target_torques_.setZero();
    target_torques_prev_.resize(action_dim_);
    target_torques_prev_.setZero();

    joint_pos_.resize(joint_num_);
    joint_pos_.setZero();

    joint_vel_.resize(joint_num_);
    joint_vel_.setZero();

    imu_ang_vel_.setZero();
    root_quat_.setIdentity();

    rclcpp::WallRate run_loop_rate_(infer_rate_);
    RCLCPP_INFO(this->get_logger(), "run_infer_loop start");
    pd_controller_flag_ = true;
    total_time_ = 0;
    std::cout << "cycle_time_: " << cycle_time_ << std::endl;
    std::cout << "infer_rate_: " << infer_rate_ << std::endl;
    std::cout << "eps_: " << eps_ << std::endl;
    std::cout << "obs_dim_: " << obs_dim_ << std::endl;
    std::cout << "action_dim_: " << action_dim_ << std::endl;
    std::cout << "stack_size_: " << stack_size_ << std::endl;
    std::cout << "obs_buffer_.size(): " << obs_buffer_.size() << std::endl;
    std::cout << "obs_input_.size(): " << obs_input_.size() << std::endl;
    std::cout << "action_output_.size(): " << action_output_.size() << std::endl;
   
    std::cout << "joint_pos_.size(): " << joint_pos_.size() << std::endl;
    std::cout << "joint_vel_.size(): " << joint_vel_.size() << std::endl;
    std::cout << "imu_ang_vel_.size(): " << imu_ang_vel_.size() << std::endl;
    std::cout << "default_dof_pos_: " << default_dof_pos_.transpose() << std::endl;
    std::cout << "joint_kp_: " << joint_kp_.transpose() << std::endl;
    std::cout << "joint_kd_: " << joint_kd_.transpose() << std::endl;
    for(int i = 0; i < joint_num_; i++)
    {
        std::cout << "joint_names_[" << i << "]: " << joint_names_[i] << std::endl;
    }
    std::cout << "joint_names_.size(): " << joint_names_.size() << std::endl;
    std::cout << "joint_num_: " << joint_num_ << std::endl;
    std::cout << "action_scales_: " << action_scales_ << std::endl;
    std::cout << "clip_observations_: " << clip_observations_ << std::endl;
    std::cout << "clip_actions_: " << clip_actions_ << std::endl;
    std::cout << "obs_scales_[\"lin_vel\"]: " << obs_scales_["lin_vel"] << std::endl;
    std::cout << "obs_scales_[\"ang_vel\"]: " << obs_scales_["ang_vel"] << std::endl;
    std::cout << "obs_scales_[\"dof_pos\"]: " << obs_scales_["dof_pos"] << std::endl;
    std::cout << "obs_scales_[\"dof_vel\"]: " << obs_scales_["dof_vel"] << std::endl;
    std::cout << "obs_scales_[\"quat\"]: " << obs_scales_["quat"] << std::endl;

    // run your control algorithm
    while (rclcpp::ok() && run_)
    {

        total_time_ += 1 / infer_rate_ ;

        sin_phase_ = std::sin(2 * (M_PI)*total_time_ / cycle_time_);
        cos_phase_ = std::cos(2 * (M_PI)*total_time_ / cycle_time_);

        contact_schedule_ = smooth_sqr_wave(sin_phase_, eps_);

        // update obs
        update_run_obs(); // obs_buffer_ is updated

        // infer action
        infer_run_action(); // action_output_ is updated

        // publish actuatorCmds
        communication::msg::ActuatorCmds actuatorCmds;
        actuatorCmds.header.frame_id = std::string("bot_elf");
        actuatorCmds.header.stamp = this->now();
        Eigen::VectorXf target_joint_pos(joint_num_);
        target_joint_pos = default_dof_pos_;

        target_joint_pos.segment(0,12) += action_output_ * action_scales_ ;
        
        // 手臂控制
        // target_joint_pos(15) += (joint_pos_(8)-default_dof_pos_(8))* action_scales_;
        // target_joint_pos(20) += (joint_pos_(2)-default_dof_pos_(2))* action_scales_;
        // target_joint_pos(15) += (joint_pos_(11)-default_dof_pos_(11))* 0.4;
        // target_joint_pos(20) += (joint_pos_(5)-default_dof_pos_(5))* 0.4;
        
        if (error_flag_)
        {
            target_joint_pos = default_dof_pos_;
            joint_kp_.setZero();
        }

        for (int i = 0; i < joint_num_; i++)
        {
            actuatorCmds.actuators_name.emplace_back(joint_names_[i]);
            actuatorCmds.pos.emplace_back(target_joint_pos(i));
            actuatorCmds.vel.emplace_back(0);
            actuatorCmds.torque.emplace_back(0);
            actuatorCmds.kp.emplace_back(joint_kp_[i]);
            actuatorCmds.kd.emplace_back(joint_kd_[i]);
        }
        actuators_cmds_pub_ptr_->publish(actuatorCmds);
        run_loop_rate_.sleep();
    }

}

double RobotController::smooth_sqr_wave(double sin_phase, double eps)
{
    return sin_phase / std::sqrt(sin_phase * sin_phase + eps * eps);
}