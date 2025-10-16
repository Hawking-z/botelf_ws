#include<iostream>
#include<eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

int main()
{
    double dt;
    int control_decimation,estimate_decimation;
    int obs_dim,stack_size,action_dim,joint_num;
    float action_scales_;
    float clip_observations_; 
    float clip_actions_;
    YAML::Node config = YAML::LoadFile("/home/zyc/bxi_ws/src/bxi_controller/conifg/controller.yaml");

    Eigen::VectorXf joint_kp_;
    Eigen::VectorXf joint_kd_;
    Eigen::VectorXf default_dof_pos_;
    Eigen::VectorXf init_dof_pos_;
    std::unordered_map<std::string, float> obs_scales_;
    Eigen::VectorXf torque_limits_;
    Eigen::VectorXf torque_limits_low_;
    Eigen::VectorXf torque_limits_high_;

    std::vector<std::string> joint_names_;

    dt = config["dt"].as<double>();
    control_decimation = config["infer_decimation"].as<int>();
    estimate_decimation = config["estimate_decimation"].as<int>();
    obs_dim = config["obs_dim"].as<int>();
    stack_size = config["stack_size"].as<int>();
    action_dim = config["action_dim"].as<int>();
    joint_num = config["joint_num"].as<int>();
    action_scales_ = config["action_scales"].as<float>();
    clip_observations_ = config["clip_observations"].as<float>();
    clip_actions_ = config["clip_actions"].as<float>();

    joint_kd_.resize(joint_num);
    joint_kp_.resize(joint_num);
    default_dof_pos_.resize(joint_num);
    init_dof_pos_.resize(joint_num);
    torque_limits_.resize(joint_num);
    torque_limits_low_.resize(joint_num);
    torque_limits_high_.resize(joint_num);
    obs_scales_["lin_vel"] = config["obs_scales"]["lin_vel"].as<float>();
    obs_scales_["ang_vel"] = config["obs_scales"]["ang_vel"].as<float>();
    obs_scales_["dof_pos"] = config["obs_scales"]["dof_pos"].as<float>();
    obs_scales_["dof_vel"] = config["obs_scales"]["dof_vel"].as<float>();
    obs_scales_["quat"] = config["obs_scales"]["quat"].as<float>();
    obs_scales_["height_measurements"] = config["obs_scales"]["height_measurements"].as<float>();

    size_t idx = 0;
    for (const auto& row : config["joint_kp"]) {
        for (const auto& value : row) {
            joint_kp_(idx++) = value.as<float>();
        }
    }
    idx = 0;
    for (const auto& row : config["joint_kd"]) {
        for (const auto& value : row) {
            joint_kd_(idx++) = value.as<float>();
        }
    }
    idx = 0;
    for (const auto& row : config["default_dof_pos"]) {
        for (const auto& value : row) {
            default_dof_pos_(idx++) = value.as<float>();
        }
    }
    idx = 0;
    for (const auto& row : config["init_dof_pos"]) {
        for (const auto& value : row) {
            init_dof_pos_(idx++) = value.as<float>();
        }
    }
    idx = 0;
    for (const auto& row : config["torque_limits"]) {
        for (const auto& value : row) {
            torque_limits_(idx++) = value.as<float>();
        }
    }
    idx = 0;
    for (const auto& row : config["torque_limits_low"]) {
        for (const auto& value : row) {
            torque_limits_low_(idx++) = value.as<float>();
        }
    }
    idx = 0;
    for (const auto& row : config["torque_limits_high"]) {
        for (const auto& value : row) {
            torque_limits_high_(idx++) = value.as<float>();
        }
    }

    for (const auto& name : config["joint_names"])
    {
        joint_names_.push_back(name.as<std::string>());
    }

    std::cout<<"dt: "<<dt<<std::endl;
    std::cout<<"control_decimation: "<<control_decimation<<std::endl;
    std::cout<<"estimate_decimation: "<<estimate_decimation<<std::endl;
    std::cout<<"obs_dim: "<<obs_dim<<std::endl;
    std::cout<<"stack_size: "<<stack_size<<std::endl;
    std::cout<<"action_dim: "<<action_dim<<std::endl;
    std::cout<<"joint_num: "<<joint_num<<std::endl;
    std::cout<<"action_scales_: "<<action_scales_<<std::endl;
    std::cout<<"clip_observations_: "<<clip_observations_<<std::endl;
    std::cout<<"clip_actions_: "<<clip_actions_<<std::endl;
    
    // std::cout<<"joint_kp:\n"<<joint_kp_<<std::endl;
    // std::cout<<"joint_kd:\n"<<joint_kd_<<std::endl;
    // std::cout<<"default_dof_pos:\n"<<default_dof_pos_<<std::endl;
    // std::cout<<"init_dof_pos:\n"<<init_dof_pos_<<std::endl;
    // for(const auto& name:joint_names_)
    // {
    //     std::cout<<name<<std::endl;
    // }

    YAML::Node state_config = YAML::LoadFile("/home/zyc/bxi_ws/src/bxi_controller/conifg/init_state.yaml");
    float total_mass = state_config["total_mass"].as<float>();
    idx = 0;
    Eigen::Vector3f init_root_pos;
    for(const auto& pos: state_config["init_root_pos"])
    {
        init_root_pos[idx++] = pos.as<float>();
    }
    // Eigen::Matrix<float, 3, 2> init_foot_pos_world = state_config["init_foot_pos_world"].as<Eigen::Matrix<float, 3, 2>>();
    std::cout<<"total_mass: "<<total_mass<<std::endl;
    std::cout<<"init_root_pos: "<<init_root_pos<<std::endl;
    // std::cout<<"init_foot_pos_world: "<<init_foot_pos_world<<std::endl;

    return 0;
}
