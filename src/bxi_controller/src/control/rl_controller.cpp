#include "control/rl_controller.h"
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

    simulation_reset_client_ =
        this->create_client<communication::srv::SimulationReset>(
            topic_prefix_ + "sim_reset");

    load_config("src/bxi_controller/config/rl_controller.yaml");

    RCLCPP_INFO(this->get_logger(), "RobotController initialized");
}

void RobotController::load_config(std::string filename)
{
    std::cout << "load config file: " << filename << std::endl;
    YAML::Node config = YAML::LoadFile(filename);
    if (!config)
    {
        throw std::runtime_error("Cannot load config file: " + filename);
    }
    else if (!config["cfg_dir"] || !config["model_name"])
    {
        throw std::runtime_error("config file missing cfg_dir or model_name field!");
    }
    else if (!config["init_dof_config"])
    {
        throw std::runtime_error("config file missing init_dof_config field!");
    }
    reset_pose = config["reset_pose"].as<bool>();
    std::string model_dir = config["cfg_dir"].as<std::string>();
    std::string cfg_name = model_dir + "robot_config.yaml";
    std::string model_name = model_dir + config["model_name"].as<std::string>();

    robot_cfg_ = cfgutils::RobotConfig::FromYamlFile(cfg_name);
    obs_assembler_ptr_ = std::make_unique<cfgutils::ObsAssembler>(robot_cfg_);
    ov_model_ptr_ = std::make_unique<ovutils::OVModelIO>(model_name, dev);
    scheduler = GaitScheduler(1.0 / robot_cfg_.infer_rate, 0.2);
    scheduler.update_period(2.0*robot_cfg_.step_period,robot_cfg_.stand_rate);
    // 加载初始关节位置等参数
    const YAML::Node &nd = config["init_dof_config"];
    if (!nd.IsMap())
    {
        throw std::runtime_error("init_dof_config 必须是映射(map)");
    }
    for (auto it = nd.begin(); it != nd.end(); ++it)
    {
        const std::string joint = it->first.as<std::string>();
        const YAML::Node &obj = it->second;
        cfgutils::DofParam p;
        // 支持缺省（若某键缺失则保留默认值）
        if (obj["default_kp"])
            p.default_kp = obj["default_kp"].as<double>();
        if (obj["default_kd"])
            p.default_kd = obj["default_kd"].as<double>();
        if (obj["init_pos"])
            p.init_pos = obj["init_pos"].as<double>();
        if (obj["control_mode"])
            p.control_mode = obj["control_mode"].as<uint8_t>();
        joints_.emplace(joint, p);
    }
    // 加载初始基座位置
    if (config["init_base_pos"])
    {
        auto pos_vec = config["init_base_pos"].as<std::vector<double>>();
        if (pos_vec.size() != 3)
        {
            throw std::runtime_error(
                "init_base_pos size error! should be 3!");
        }
        init_base_pos_ = Eigen::Vector3f(pos_vec[0], pos_vec[1], pos_vec[2]);
    }
    if (config["init_base_quat"])
    {
        auto quat_vec = config["init_base_quat"].as<std::vector<double>>();
        if (quat_vec.size() != 4)
        {
            throw std::runtime_error(
                "init_base_quat size error! should be 4!");
        }
        init_base_quat_ = Eigen::Vector4f(quat_vec[0], quat_vec[1], quat_vec[2], quat_vec[3]);
        init_base_quat_.normalize();
    }

    // TODO
    if (robot_cfg_.num_actions == 12)
    {
        with_arm = false;
    }
    else if (robot_cfg_.num_actions == 20)
    {
        with_arm = true;
    }
    else
    {
        throw std::runtime_error("num_actions in robot_config.yaml error!");
    }
    action_output_.resize(robot_cfg_.num_actions);
    action_output_.setZero();

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
    const size_t n = msg->name.size();
    for (size_t i = 0; i < n; i++)
    {
        const std::string &joint_name = msg->name[i];
        auto it = joints_.find(joint_name);
        if (it != joints_.end())
        {
            it->second.pos = msg->position[i];
            it->second.vel = msg->velocity[i];
        }
    }
}

// only for simulation
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

// // 根据初始化的default_dof_pos_对关节位置进行简单的pd控制
void RobotController::init_robot_pos()
{
    rclcpp::Rate loop_rate_(250);
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

        for (auto it = joints_.begin(); it != joints_.end(); ++it)
        {
            actuatorCmds.actuators_name.emplace_back(it->first);
            actuatorCmds.vel.emplace_back(0);
            actuatorCmds.torque.emplace_back(0);
            soft_start = loop_count / 250.0;
            soft_start = soft_start > 1 ? 1 : soft_start;
            actuatorCmds.pos.emplace_back(it->second.init_pos * soft_start);
            actuatorCmds.kp.emplace_back(it->second.default_kp * soft_start);
            actuatorCmds.kd.emplace_back(it->second.default_kd);
        }
        actuators_cmds_pub_ptr_->publish(actuatorCmds);

        loop_count++;
        {
            if (loop_count > (3. * 250.0)&& ((motion_commands_msg_->mode) % 2 == 1)) // hold for 3s
            {
                break;
            }
        } // 限制lock_guard作用域
        loop_rate_.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Robot Joint_Pos Init done!");
}

void RobotController::callSimulationResetService()
{
    // 等待 service 可用
    while (!simulation_reset_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "等待 sim_reset 服务上线...");
    }

    // 构造请求
    auto request = std::make_shared<communication::srv::SimulationReset::Request>();

    request->header.frame_id = std::string("bot_elf");

    // 示例：给 base_pose 赋值
    request->base_pose.position.x = init_base_pos_[0];
    request->base_pose.position.y = init_base_pos_[1];
    request->base_pose.position.z = init_base_pos_[2];

    request->base_pose.orientation.x = init_base_quat_[0];
    request->base_pose.orientation.y = init_base_quat_[1];
    request->base_pose.orientation.z = init_base_quat_[2];
    request->base_pose.orientation.w = init_base_quat_[3];
    for (int i = 0; i < 4; i++)
    {
        std::cout << init_base_quat_[i] << std::endl;
    }
    // 示例：根据需要设置关节信息
    for (auto it = joints_.begin(); it != joints_.end(); ++it)
    {
        request->joint_state.name.emplace_back(it->first);
        request->joint_state.position.emplace_back(it->second.init_pos);
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
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "SimulationReset false: is_success=false");
        }
    };
    // 调用异步请求
    auto future_result = simulation_reset_client_->async_send_request(request, response_received_callback);
}

void RobotController::pd_controller_loop()
{
    // 等待传感器数据
    rclcpp::WallRate loop_rate_(robot_cfg_.pd_rate);
    RCLCPP_INFO(this->get_logger(), "pd_controller_loop start!");
    while (rclcpp::ok() && run_)
    {
        try
        {
            // publish actuatorCmds
            communication::msg::ActuatorCmds actuatorCmds;
            actuatorCmds.header.frame_id = std::string("bot_elf");
            actuatorCmds.header.stamp = this->now();
            {
                for (auto it = joints_.begin(); it != joints_.end(); ++it)
                {
                    auto control_joint = robot_cfg_.dof.name_to_index.find(it->first);
                    if (control_joint != robot_cfg_.dof.name_to_index.end())
                    {
                        actuatorCmds.actuators_name.emplace_back(it->first);
                        int index = robot_cfg_.dof.name_to_index[it->first];
                        // TODO 加入action
                        // 位置控制 
                        if (it->second.control_mode == 0)
                        {
                            float traget_pos = action_output_[index] * robot_cfg_.dof.scale[index] + robot_cfg_.dof.default_pos[index] ;
                            actuatorCmds.pos.emplace_back(traget_pos);
                            actuatorCmds.vel.emplace_back(0);
                            actuatorCmds.torque.emplace_back(0);
                            actuatorCmds.kp.emplace_back(robot_cfg_.dof.kp[index]);
                            actuatorCmds.kd.emplace_back(robot_cfg_.dof.kd[index]);
                        }
                        // 力矩控制
                        else if (it->second.control_mode == 1)
                        {
                            float target_torque = robot_cfg_.dof.kp[index] *(action_output_[index] * robot_cfg_.dof.scale[index] 
                                                + robot_cfg_.dof.default_pos[index] - it->second.pos)
                                                + robot_cfg_.dof.kd[index] * (0.0 - it->second.vel) ;
                            actuatorCmds.pos.emplace_back(0);
                            actuatorCmds.vel.emplace_back(0);
                            actuatorCmds.kp.emplace_back(0);
                            actuatorCmds.kd.emplace_back(0);
                            actuatorCmds.torque.emplace_back(target_torque);
                        }
                        else
                        {
                            throw std::runtime_error("unknown control mode !!!!");
                        }
                    }
                    else
                    {
                        actuatorCmds.actuators_name.emplace_back(it->first);
                        actuatorCmds.pos.emplace_back(it->second.init_pos);
                        actuatorCmds.vel.emplace_back(0);
                        actuatorCmds.torque.emplace_back(0);
                        actuatorCmds.kp.emplace_back(it->second.default_kp);
                        actuatorCmds.kd.emplace_back(it->second.default_kd);
                    }
                }
                actuators_cmds_pub_ptr_->publish(actuatorCmds);
            }
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
    {   
        auto sensor_input = obs_assembler_ptr_->sensor_input();
        Eigen::Quaternionf root_quat;
        Eigen::Vector3f root_euler;
        {
            std::lock_guard<std::mutex> lock1(imu_mutex_);
            sensor_input["base_ang_vel"](0) = imu_msg_->angular_velocity.x;
            sensor_input["base_ang_vel"](1) = imu_msg_->angular_velocity.y;
            sensor_input["base_ang_vel"](2) = imu_msg_->angular_velocity.z;
            root_quat.x() = imu_msg_->orientation.x;
            root_quat.y() = imu_msg_->orientation.y;
            root_quat.z() = imu_msg_->orientation.z;    
            root_quat.w() = imu_msg_->orientation.w;
            root_euler = get_euler_xyz(root_quat);
            sensor_input["base_euler_xyz"](0) = root_euler(0);
            sensor_input["base_euler_xyz"](1) = root_euler(1);
            sensor_input["base_euler_xyz"](2) = root_euler(2);
        }
        {
            std::lock_guard<std::mutex> lock2(joint_mutex_);
            for(const auto& kv : robot_cfg_.dof.name_to_index)
            {
                int index = kv.second;
                const cfgutils::DofParam& dof_param = joints_.at(kv.first);
                sensor_input["dof_pos"](index) = dof_param.pos - robot_cfg_.dof.default_pos[index] ;
                sensor_input["dof_vel"](index) = dof_param.vel;
            }
        }
        {
            std::lock_guard<std::mutex> lock3(motion_commands_mutex_);
            sensor_input["command_lin_vel"](0) = motion_commands_msg_->vel_des.x;
            sensor_input["command_lin_vel"](1) = motion_commands_msg_->vel_des.y;
            sensor_input["command_ang_vel"](0) = motion_commands_msg_->yawdot_des;
        }
        scheduler.update_phase();
        sensor_input["sin_phase"](0) = scheduler.l_sin_phase;
        sensor_input["sin_phase"](1) = scheduler.r_sin_phase;
        sensor_input["actions"] = action_output_;
    
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
        obs_dict_ = obs_assembler_ptr_->step(sensor_input);
        // obs_assembler_ptr_->print_obs_dict(obs_dict_);
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
        for(const auto& kv : obs_assembler_ptr_->obs_buffers())
        {
            const auto& name = kv.first;
            auto& obs_data = obs_dict_.at(name);
            const size_t D = static_cast<size_t>(obs_data.size());
            auto& t = ov_model_ptr_->tensor(name);
            float* p = t.data<float>();
            std::memcpy(p,obs_data.data(),sizeof(float)*D);
            // ov_model_ptr_->infer_request().set_input_tensor(name, ov_model_ptr_->tensor(name));
        }
        ov_model_ptr_->infer_request().start_async();
        ov_model_ptr_->infer_request().wait();
        // Get output tensor & set action_output_
        auto output_tensor = ov_model_ptr_->infer_request().get_output_tensor();
        auto output_buf = output_tensor.data<const float>();
        for (int i = 0; i < robot_cfg_.num_actions; ++i)
        {
            action_output_(i) = output_buf[i];
        }
        action_output_ = action_output_.cwiseMin(robot_cfg_.clip_actions).cwiseMax(-robot_cfg_.clip_actions);
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(this->get_logger(), "infer_action error");
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void RobotController::infer_loop()
{
    rclcpp::WallRate loop_rate_(robot_cfg_.infer_rate);
    RCLCPP_INFO(this->get_logger(), "infer_loop start");
    pd_controller_flag_ = true;

    // run your control algorithm
    while (rclcpp::ok() && run_)
    {

        // update obs
        update_obs(); 

        // infer action
        infer_action(); // action_output_ is updated
 
        if (error_flag_)
        {
        }

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
        if (topic_prefix_.find("simulation") != std::string::npos && reset_pose)
            callSimulationResetService();
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(this->get_logger(), "main_thread error");
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
    std::this_thread::sleep_for(5ms);
    infer_loop_thread_ = std::thread(&RobotController::infer_loop, this);
    pd_controller_thread_ = std::thread(&RobotController::pd_controller_loop, this);
}
