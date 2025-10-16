#include "control/RL_Controller.h"
#include <rcpputils/asserts.hpp>
#include <yaml-cpp/yaml.h>

// TODO 替换为std::remainder(angle, 2 * M_PI)

RobotController::RobotController() : Node("RobotController")
{

    this->declare_parameter("/topic_prefix", "simulation/");

    topic_prefix_ = this->get_parameter("/topic_prefix")
                        .get_parameter_value()
                        .get<std::string>();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    actuators_cmds_pub_ptr_ =
        this->create_publisher<communication::msg::ActuatorCmds>(
            topic_prefix_ + "actuators_cmds", qos);

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

    load_config("src/bxi_controller/config/rl_controller.yaml");

    obs_cur_.resize(obs_dim_);
    obs_cur_.setZero();
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

    RCLCPP_INFO(this->get_logger(), "RobotController initialized");
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
    init_dof_pos_.resize(joint_num_);
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
    for (const auto &row : config["init_dof_pos"])
    {
        for (const auto &value : row)
        {
            init_dof_pos_(idx++) = value.as<float>();
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

    for (const auto &name : config["joint_names"])
    {
        joint_names_.push_back(name.as<std::string>());
    }

    std::string ov_model_path_ = config["model"].as<std::string>();
    ov::Core ov_core_;
    ov_compiled_model_ = ov_core_.compile_model(ov_model_path_, "CPU");
    auto ov_input_port = ov_compiled_model_.input();
    ov_input_type_ = ov_input_port.get_element_type();
    ov_input_shape_ = ov_input_port.get_shape();
    ov_infer_request_ = ov_compiled_model_.create_infer_request();

    RCLCPP_INFO(this->get_logger(), "param init !");
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

void RobotController::commands_callback(const communication::msg::MotionCommands::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(motion_commands_mutex_);
    motion_commands_msg_ = msg;
}

void RobotController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_msg_ = msg;
}

void RobotController::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_mutex_);
    joint_state_msg_ = msg;
}

void RobotController::touch_callback(const communication::msg::TouchSensor::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(touch_mutex_);
    touch_sensor_msg_ = msg;
}
// only for simulation
void RobotController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_msg_ = msg;
}

void RobotController::init()
{
    main_thread_ = std::thread(&RobotController::main_thread, this);
    run_ = true;
}

RobotController::~RobotController()
{
    run_ = false;
    if (main_thread_.joinable())
    {
        main_thread_.join();
        std::cout << "main_thread_ joined" << std::endl;
    }

    if (pd_controller_thread_.joinable())
    {
        pd_controller_thread_.join();
        std::cout << "pd_controller_thread_ joined" << std::endl;
    }

    if (infer_loop_thread_.joinable())
    {
        infer_loop_thread_.join();
        std::cout << "infer_loop_thread_ joined" << std::endl;
    }
}

// 根据初始化的default_dof_pos_对关节位置进行简单的pd控制
void RobotController::init_robot_pos()
{
    rclcpp::Rate loop_rate_(pd_rate_);
    RCLCPP_INFO(this->get_logger(), "Robot Joint_Pos Init start!");
    size_t loop_count = 0;
    double soft_start = 0;
    // init motor pos
    while (rclcpp::ok() && run_)
    {
        auto motion_commands_msg = motion_commands_msg_;
        communication::msg::ActuatorCmds actuatorCmds;
        actuatorCmds.header.frame_id = std::string("bot_elf");
        actuatorCmds.header.stamp = this->now();

        update_obs();

        for (int i = 0; i < joint_num_; i++)
        {
            actuatorCmds.actuators_name.emplace_back(joint_names_[i]);
            actuatorCmds.pos.emplace_back(init_dof_pos_(i));
            actuatorCmds.vel.emplace_back(0);
            actuatorCmds.torque.emplace_back(0);

            soft_start = loop_count / pd_rate_;
            soft_start = soft_start > 1 ? 1 : soft_start;

            actuatorCmds.kp.emplace_back(joint_kp_(i) * soft_start);
            actuatorCmds.kd.emplace_back(joint_kd_(i));
        }

        actuators_cmds_pub_ptr_->publish(actuatorCmds);

        loop_count++;
        {
            if (loop_count > (3. * pd_rate_) && ((motion_commands_msg_->mode) % 2 == 1)) // hold for 3s
            {
                break;
            }
        } // 限制lock_guard作用域
        loop_rate_.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Robot Joint_Pos Init done!");
}

void RobotController::pd_controller_loop()
{
    // 等待传感器数据
    rclcpp::WallRate loop_rate_(pd_rate_);
    RCLCPP_INFO(this->get_logger(), "pd_controller_loop start!");
    while (rclcpp::ok() && run_)
    {
        try
        {
            // // publish actuatorCmds
            // communication::msg::ActuatorCmds actuatorCmds;
            // actuatorCmds.header.frame_id = std::string("bot_elf");
            // actuatorCmds.header.stamp = this->now();

            // Eigen::VectorXf target_joint_pos(action_dim_);
            // {
            //     target_joint_pos = action_output_ * action_scales_ + default_dof_pos_.head(12);
            //     //  下半身
            //     for (int i = 0; i < action_dim_; i++)
            //     {
            //         actuatorCmds.actuators_name.emplace_back(joint_names_[i]);
            //         actuatorCmds.pos.emplace_back(target_joint_pos(i));
            //         actuatorCmds.vel.emplace_back(0);
            //         actuatorCmds.torque.emplace_back(0);
            //         actuatorCmds.kp.emplace_back(joint_kp_[i]);
            //         actuatorCmds.kd.emplace_back(joint_kd_[i]);
            //     }
            //     // 上半身
            //     for (int i = action_dim_; i < joint_num_; i++)
            //     {
            //         actuatorCmds.actuators_name.emplace_back(joint_names_[i]);
            //         actuatorCmds.pos.emplace_back(default_dof_pos_(i));
            //         actuatorCmds.vel.emplace_back(0);
            //         actuatorCmds.torque.emplace_back(0);

            //         actuatorCmds.kp.emplace_back(joint_kp_[i]);
            //         actuatorCmds.kd.emplace_back(joint_kd_[i]);
            //     }
            //     actuators_cmds_pub_ptr_->publish(actuatorCmds);
            // }
            loop_rate_.sleep();
        }
        catch (const std::exception &e)
        {
            RCLCPP_INFO(this->get_logger(), "pd_controller_loop error");
            RCLCPP_ERROR(this->get_logger(), e.what());
        }
    }
}

// 根据传感器数据和指令数据更新观测数据obs_buffer
void RobotController::update_obs()
{
    try
    {   obs_cur_.setZero();
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
                RCLCPP_ERROR(this->get_logger(), "RobotController: Robot fall down!");
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

void RobotController::infer_action()
{
    try
    {
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

void RobotController::infer_loop()
{
    // 等待传感器数据

    rclcpp::WallRate loop_rate_(infer_rate_);
    RCLCPP_INFO(this->get_logger(), "infer_loop start");
    pd_controller_flag_ = true;
    // 初始化时间点
    total_time_ = 0;

    // run your control algorithm
    while (rclcpp::ok() && run_)
    {

        total_time_ += 1 / infer_rate_ ;

        sin_phase_ = std::sin(2 * (M_PI)*total_time_ / cycle_time_);
        cos_phase_ = std::cos(2 * (M_PI)*total_time_ / cycle_time_);

        contact_schedule_ = smooth_sqr_wave(sin_phase_, eps_);

        // update obs
        update_obs(); // obs_buffer_ is updated

        // infer action
        infer_action(); // action_output_ is updated

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

        //  下半身
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
        loop_rate_.sleep();
    }
}

void RobotController::main_thread()
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (imu_msg_ == nullptr || joint_state_msg_ == nullptr || motion_commands_msg_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Sensor data not ready!");
        return;
    }
    try
    {
        // Enable motor position control
        reset_robot(1, false);
        // init robot joint pos while hang up
        init_robot_pos();
        // release robt(only for simulation), enable motor position, velocity and torque control
        reset_robot(2, true);
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(this->get_logger(), "main_thread error");
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
    // std::this_thread::sleep_for(5ms);
    infer_loop_thread_ = std::thread(&RobotController::infer_loop, this);
    // pd_controller_thread_ = std::thread(&RobotController::pd_controller_loop, this);
}

double RobotController::smooth_sqr_wave(double sin_phase, double eps)
{
    return sin_phase / std::sqrt(sin_phase * sin_phase + eps * eps);
}
