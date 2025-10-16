#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#define NUM_LEG 2

#include <iostream>

class RobotState
{
public:
    void reset(std::string config_path,int dof_num)
    {
        YAML::Node config = YAML::LoadFile(config_path);
        for(int i = 0; i < 3; i++)
        {
            init_root_pos(i) = config["init_root_pos"][i].as<double>();
        }
       
        for(int i = 0; i < NUM_LEG; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                init_foot_pos_world(j, i) = config["init_foot_pos_world"][i][j].as<double>();
            }
        }
        init_dof_pos_.resize(dof_num);

        int idx = 0;
        for(const auto& row : config["init_dof_pos"])
        {
            for(const auto& value : row)
            {
                init_dof_pos_(idx++) = value.as<double>();
            }
        }
        
        // 初始化
        // 传感器数据
        imu_acc.setZero();
        imu_ang_vel.setZero();
        joint_pos.resize(dof_num);
        joint_vel.resize(dof_num);
        joint_pos.lazyAssign(init_dof_pos_);
        joint_vel.setZero();
        
        // 重要的运动学变量
        root_pos = init_root_pos;
        root_quat.setIdentity();
        root_euler.setZero();
        root_rot_mat.setIdentity();
        root_lin_vel.setZero();

        world_lin_vel.setZero();

        foot_pos_abs.resize(NUM_LEG);
        foot_vel_abs.resize(NUM_LEG);
        foot_quat_world.resize(NUM_LEG);

        for(int i = 0; i < NUM_LEG; i++)
        {
            foot_pos_abs[i] = init_root_pos - init_foot_pos_world.col(i);
            foot_vel_abs[i].setZero();
            foot_quat_world[i].setIdentity();
        }
        

        com_pos.setZero();
        com_vel.setZero();
        P_com.setZero();
        L_contact_mea.setZero();
    }

    // IMU sensor data
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_ang_vel;
    
    // importance kinematic variavles
    Eigen::Vector3d root_pos;
    Eigen::Vector3d root_lin_vel;
    Eigen::Quaterniond root_quat;

    Eigen::Vector3d world_lin_vel;
    // rpy
    Eigen::Vector3d root_euler;
    Eigen::Matrix3d root_rot_mat;

    Eigen::VectorXd joint_pos;
    Eigen::VectorXd joint_vel;

    // touch force
    Eigen::Vector2d touch_force;
    
    // in a frame which centered at the robot frame's origin but parallels to the world frame
    // Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs;  
    // Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs;
    // Eigen::Matrix<double, 4, NUM_LEG> foot_quat_world;
    std::vector<Eigen::Vector3d> foot_pos_abs;
    std::vector<Eigen::Vector3d> foot_vel_abs;
    std::vector<Eigen::Quaterniond> foot_quat_world;
   
    
    double total_mass;

    // in world frame
    Eigen::Vector3d com_pos;
    Eigen::Vector3d com_vel;
    Eigen::Vector3d P_com; // linear momentum
    Eigen::Vector3d L_com; // angular momentum
    // 关于接触点的角动量 在世界坐标
    Eigen::Vector3d L_contact_mea;
    double Lx_est = 0;
    double Ly_est = 0;

    double Lx_est_f = 0;
    double Ly_est_f = 0;

    double px_f = 0;
    double py_f = 0;

    //init pos 
    Eigen::Vector3d init_root_pos; 
    Eigen::Matrix<double, 3, NUM_LEG> init_foot_pos_world;
    Eigen::VectorXd init_dof_pos_;

    int mode = 0;
};