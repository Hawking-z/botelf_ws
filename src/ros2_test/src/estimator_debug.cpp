

#include <eigen3/Eigen/Dense>
// pinocchio
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "rclcpp/rclcpp.hpp"

#include <yaml-cpp/yaml.h>


#include <std_msgs/msg/float32_multi_array.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <communication/msg/touch_sensor.hpp>
#include <communication/srv/robot_reset.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <communication/msg/actuator_cmds.hpp>
#include <communication/msg/motion_commands.hpp>


#include <mutex>
#include <eigen3/Eigen/Dense>

#include <deque>
#include <unordered_map>
#include <cmath>
#include <thread>
#include <chrono>


using namespace rclcpp;
using namespace std::chrono_literals;

// state estimator parameters
#define STATE_SIZE 12
#define MEAS_SIZE 14
#define PROCESS_NOISE_PIMU 0.01
#define PROCESS_NOISE_VIMU 0.01
#define PROCESS_NOISE_PFOOT 0.01
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1
#define SENSOR_NOISE_ZFOOT 0.001
#define MAX_NUM 1000
#define NUM_LEG 2


double wrap_to_pi(double angle)
{
    // 使用 std::remainder(angle, 2 * M_PI) 归一化到 [-π, π]
    return std::remainder(angle, 2.0 * M_PI);
}

Eigen::Vector3d wrap_to_pi(const Eigen::Vector3d &angles)
{
    // 使用 std::remainder(angle, 2 * M_PI) 归一化到 [-π, π]
    double x = std::remainder(angles[0], 2.0 * M_PI);
    double y = std::remainder(angles[1], 2.0 * M_PI);
    double z = std::remainder(angles[2], 2.0 * M_PI);
    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d quat_rotate(const Eigen::Quaterniond &quat, const Eigen::Vector3d &point)
{
    // 使用四元数旋转点，quat * point * quat.inverse() 简化为 Eigen 的旋转操作
    return quat * point;
}

Eigen::Vector3d quat_rotate_yaw(const Eigen::Quaterniond &quat, const Eigen::Vector3d &vec)
{
    // 提取四元数的分量（w, x, y, z）
    double w = quat.w();
    double x = 0.0; // 强制置零
    double y = 0.0; // 强制置零
    double z = quat.z();

    // 构造仅包含 yaw 分量的四元数
    Eigen::Quaterniond quat_yaw(w, x, y, z);
    quat_yaw.normalize(); // 归一化，确保是单位四元数

    // 使用新的 yaw 四元数旋转向量
    return quat_yaw * vec;
}

// 函数：使用四元数对向量进行逆旋转
Eigen::Vector3d quat_rotate_inverse(const Eigen::Quaterniond &quat, const Eigen::Vector3d &vec)
{
    // 使用四元数的共轭（表示逆旋转）
    Eigen::Quaterniond quat_inverse = quat.conjugate();

    // 对向量执行逆旋转
    return quat_inverse * vec;
}

// 函数：使用四元数的 yaw 分量对向量应用逆旋转
Eigen::Vector3d quat_rotate_yaw_inverse(const Eigen::Quaterniond &quat, const Eigen::Vector3d &vec)
{
    // 提取四元数的分量（w, x, y, z）
    double w = quat.w();
    double x = 0.0; // 强制置零
    double y = 0.0; // 强制置零
    double z = quat.z();

    // 构造仅包含 yaw 分量的四元数
    Eigen::Quaterniond quat_yaw(w, x, y, z);
    quat_yaw.normalize(); // 归一化，确保是单位四元数

    // 计算 yaw 的逆旋转（使用共轭）
    Eigen::Quaterniond quat_yaw_inverse = quat_yaw.conjugate();

    // 对向量应用逆旋转
    return quat_yaw_inverse * vec;
}

Eigen::Vector3d get_euler_xyz(const Eigen::Quaterniond &quat)
{
    // 提取四元数分量
    double qw = quat.w(), qx = quat.x(), qy = quat.y(), qz = quat.z();

    // 计算 roll (绕 x 轴旋转)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = qw * qw - qx * qx - qy * qy + qz * qz;
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // 计算 pitch (绕 y 轴旋转)
    double sinp = 2.0 * (qw * qy - qz * qx);
    double pitch;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2.0, sinp); // 限制在 [-π/2, π/2]
    else
        pitch = std::asin(sinp);

    // 计算 yaw (绕 z 轴旋转)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = qw * qw + qx * qx - qy * qy - qz * qz;
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    // 归一化到 [-π, π]
    roll = std::remainder(roll, 2.0 * M_PI);
    pitch = std::remainder(pitch, 2.0 * M_PI);
    yaw = std::remainder(yaw, 2.0 * M_PI);

    // 返回 Eigen::Vector3d
    return Eigen::Vector3d(roll, pitch, yaw);
}

double get_yaw_from_quaternion(const Eigen::Quaterniond &quat)
{
    // 提取四元数分量
    double qw = quat.w(), qx = quat.x(), qy = quat.y(), qz = quat.z();

    // 计算 yaw (绕 z 轴的旋转角)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = qw * qw + qx * qx - qy * qy - qz * qz;

    // atan2 计算 yaw，并归一化到 [-π, π]
    return std::remainder(std::atan2(siny_cosp, cosy_cosp), 2.0 * M_PI);
}

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

class Estimator
{
public:
    Estimator();
    void init_state(RobotState& state);
    void update_estimate(RobotState &state,double * estimated_contacts,int contact_index, double dt) ;
private:
    Eigen::Matrix<double, STATE_SIZE, 1> x; // estimation state
    Eigen::Matrix<double, STATE_SIZE, 1> xbar; // estimation state after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; // estimation state covariance
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // estimation state covariance after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A; // estimation state transition
    Eigen::Matrix<double, STATE_SIZE, 3> B; // estimation state transition
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; // estimation state transition noise

    // observation
    // 0 1 2   L pos residual
    // 3 4 5   R pos residual
    // 6 7 8 vel residual from L
    // 9 10 11 vel residual from R
    // 12 13  foot height
    Eigen::Matrix<double, MEAS_SIZE, 1> y; //  observation
    Eigen::Matrix<double, MEAS_SIZE, 1> yhat; // estimated observation
    Eigen::Matrix<double, MEAS_SIZE, 1> error_y; // estimated observation
    Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y; // S^-1*error_y
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C; // estimation state observation
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC; // S^-1*C
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R; // estimation state observation noise
    // helper matrices
    Eigen::Matrix3d identity_3x3 = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S; // Innovation (or pre-fit residual) covariance
    Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K; // kalman gai

    // pino
    pinocchio::Model model_biped;

    // left 0 right 1
    pinocchio::FrameIndex foot_end_idx[2];
    pinocchio::Data data_biped;

    // kalman filter for angular momentum
    double Cov_px_ = 0.01;
    double Cov_py_ = 0.01;
    double Cov_Lx_ = 0.25;
    double Cov_Ly_ = 0.25;

    double sigma_Lx_ = 1;
    double sigma_Ly_ = 1;

    // configuration
    Eigen::VectorXd q;
    Eigen::VectorXd v;

};

Estimator::Estimator()
{
    C.setZero();
    for (int i = 0; i < NUM_LEG; i++)
    {
        C.block<3, 3>(i * 3, 0) = identity_3x3;
        C.block<3, 3>(i * 3, 6 + i * 3) = -identity_3x3;
        C.block<3, 3>(NUM_LEG * 3 + i * 3, 3) = identity_3x3;
        C(NUM_LEG * 6 + i, 6 + i * 3 + 2) = 1;
    }

    Q.setIdentity();
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * identity_3x3;
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * identity_3x3;
    for (int i = 0; i < NUM_LEG; i++)
    {
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) = PROCESS_NOISE_PFOOT * identity_3x3;
    }

    R.setIdentity();
    for (int i = 0; i < NUM_LEG; i++)
    {
        R.block<3, 3>(i * 3, i * 3) = SENSOR_NOISE_PIMU_REL_FOOT * identity_3x3;
        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = SENSOR_NOISE_VIMU_REL_FOOT * identity_3x3;
        R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = SENSOR_NOISE_ZFOOT;
    }

    // set A to identity
    A.setIdentity();

    // set B to zero
    B.setZero();

    const std::string urdf_robot_filename = std::string("src/bxi_controller/config/model/bot_elf/urdf/bot_elf_new.urdf");

    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(urdf_robot_filename, root_joint, model_biped);

    auto l_ankle_Joint = model_biped.getJointId("l_ankle_x_joint");
    auto r_ankle_Joint = model_biped.getJointId("r_ankle_x_joint");

    pinocchio::SE3 FE_Off;
    Eigen::Matrix3d R;                 // 新增足底坐标系的偏置旋转矩阵
    Eigen::Vector3d p = {0, 0, -0.04}; // 新增足底坐标系的偏置位移向量

    R.setIdentity();
    FE_Off.rotation_impl(R);
    FE_Off.translation_impl(p);
    pinocchio::Frame leftFootEnd("left_footend", l_ankle_Joint, 0, FE_Off, pinocchio::FIXED_JOINT, pinocchio::Inertia::Zero());
    pinocchio::Frame rightFootEnd("right_footend", r_ankle_Joint, 0, FE_Off, pinocchio::FIXED_JOINT, pinocchio::Inertia::Zero());
    model_biped.addFrame(leftFootEnd);
    model_biped.addFrame(rightFootEnd);

    foot_end_idx[0] = model_biped.getFrameId("left_footend");
    foot_end_idx[1] = model_biped.getFrameId("right_footend");
    data_biped = pinocchio::Data(model_biped);
    q.resize(model_biped.nq);
    v.resize(model_biped.nv);
    q.setZero();
    v.setZero();
}

void Estimator::init_state(RobotState &state)
{
    x.setZero();
    x.segment<3>(0) = state.init_root_pos;
    for (int i = 0; i < NUM_LEG; i++)
    {
        x.segment<3>(6 + i * 3) = state.init_foot_pos_world.col(i);
    }
    P.setIdentity();
    P = P * 3;
    state.total_mass = pinocchio::computeTotalMass(model_biped, data_biped);
    std::cout << "total mass: " << state.total_mass << std::endl;
}

void Estimator::update_estimate(RobotState &state, double *estimated_contacts, int contact_index, double dt)
{
    state.root_rot_mat = state.root_quat.toRotationMatrix();
    
   
    // forward kinematics
    // root pos 和 root lin vel 为 0
    q(0) = 0.0;
    q(1) = 0.0;
    q(2) = 0.0;
    v(0) = 0.0;
    v(1) = 0.0;
    v(2) = 0.0;
    // base_lin_vel: tensor([[-0.0019, -0.0007, -0.2043]], device='cuda:0')
    // v(0) = -0.0019;  v(1) = -0.0007;  v(2) = -0.2043;

    q(3) = state.root_quat.x();
    q(4) = state.root_quat.y();
    q(5) = state.root_quat.z();
    q(6) = state.root_quat.w();

    Eigen::VectorXd joint_pos_reordered(state.joint_pos.size()); // 按照新顺序重组
    Eigen::VectorXd joint_vel_reordered(state.joint_vel.size()); // 按照新顺序重组
    joint_pos_reordered << state.joint_pos.segment(0, 6), state.joint_pos.segment(12, 4),
        state.joint_pos.segment(6, 6), state.joint_pos.segment(16, 4);

    joint_vel_reordered << state.joint_vel.segment(0, 6), state.joint_vel.segment(12, 4),
        state.joint_vel.segment(6, 6), state.joint_vel.segment(16, 4);

    q.tail(model_biped.nq - 7) = joint_pos_reordered;

    v.block(3, 0, 3, 1) = state.imu_ang_vel;
    v.tail(model_biped.nv - 6) = joint_vel_reordered;

    pinocchio::forwardKinematics(model_biped, data_biped, q, v);
    pinocchio::computeCentroidalMomentum(model_biped, data_biped, q, v);
    pinocchio::updateFramePlacements(model_biped, data_biped);

    for (int i = 0; i < NUM_LEG; i++)
    {
        Eigen::Vector3d fk_pos = data_biped.oMf[foot_end_idx[i]].translation();
        Eigen::Vector3d leg_v = pinocchio::getFrameVelocity(model_biped, data_biped, foot_end_idx[i], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED).linear();
        Eigen::Matrix3d fk_quat = data_biped.oMf[foot_end_idx[i]].rotation();

        state.foot_pos_abs[i] = fk_pos;
        state.foot_vel_abs[i] = leg_v;
        state.foot_quat_world[i] = Eigen::Quaterniond(fk_quat);
    }
    // update A B using latest dt
    A.block<3, 3>(0, 3) = dt * identity_3x3;
    B.block<3, 3>(0, 0) = 0.5 * dt * dt * identity_3x3;
    B.block<3, 3>(3, 0) = dt * identity_3x3;

    double gravity = 9.81;
    Eigen::Vector3d gravity_vec = {0, 0, -gravity};
    // control input
    Eigen::Vector3d u = state.root_rot_mat * state.imu_acc + gravity_vec;

    // for(int i = 0; i < NUM_LEG; i++)
    // {
    //     estimated_contacts[i] = std::min(std::max((state.touch_force(i)) / (150.0 - 0.0), 0.0), 1.0);
    // }

    // update Q
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * identity_3x3;
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.81 / 20.0 * identity_3x3;

    // update Q R for legs not in contact
    for (int i = 0; i < NUM_LEG; ++i)
    {
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) = (1 + (1 - estimated_contacts[i]) * MAX_NUM) * dt * PROCESS_NOISE_PFOOT * identity_3x3; // foot position transition

        R.block<3, 3>(i * 3, i * 3) = (1 + (1 - estimated_contacts[i]) * MAX_NUM) * SENSOR_NOISE_PIMU_REL_FOOT *
                                      identity_3x3;
        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = (1 + (1 - estimated_contacts[i]) * MAX_NUM) * SENSOR_NOISE_VIMU_REL_FOOT * identity_3x3; // vel estimation
        R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = (1 + (1 - estimated_contacts[i]) * MAX_NUM) * SENSOR_NOISE_ZFOOT;                                            // height z estimation
    }

    // process update
    xbar = A * x + B * u;
    Pbar = A * P * A.transpose() + Q;

    yhat = C * xbar;

    for (int i = 0; i < NUM_LEG; ++i)
    {
        Eigen::Vector3d fk_pos = state.foot_pos_abs[i];
        y.block<3, 1>(i * 3, 0) = -fk_pos; // fk estimation
        y.block<3, 1>(NUM_LEG * 3 + i * 3, 0) =
            (1.0 - estimated_contacts[i]) * x.segment<3>(3) - estimated_contacts[i] * state.foot_vel_abs[i]; // vel estimation
        y(NUM_LEG * 6 + i) = (1.0 - estimated_contacts[i]) * (x(2) + fk_pos(2));                             // height z estimation
    }

    S = C * Pbar * C.transpose() + R;
    S = 0.5 * (S + S.transpose());

    error_y = y - yhat;
    Serror_y = S.fullPivHouseholderQr().solve(error_y);

    x = xbar + Pbar * C.transpose() * Serror_y;
    // S^-1*C
    SC = S.fullPivHouseholderQr().solve(C);
    P = Pbar - Pbar * C.transpose() * SC * Pbar;
    P = 0.5 * (P + P.transpose());

    // reduce position drift
    // if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
    //     P.block<2, 10>(0, 2).setZero();
    //     P.block<10, 2>(2, 0).setZero();
    //     P.block<2, 2>(0, 0) /= 10.0;
    // }

    state.root_pos = x.segment<3>(0);
    // state.root_lin_vel = state.root_rot_mat.transpose() * x.segment<3>(3);
    Eigen::Vector3d root_lin_vel_world = x.segment<3>(3);
    state.root_lin_vel = state.root_rot_mat.transpose() * root_lin_vel_world;
    state.world_lin_vel = root_lin_vel_world;
    state.com_pos = state.root_pos + data_biped.com[0];
    state.com_vel = root_lin_vel_world + data_biped.vcom[0];
    state.P_com = state.total_mass * state.com_vel;

    state.L_com = data_biped.hg.angular();

    Eigen::Vector3d com_rel_contact = data_biped.com[0] - state.foot_pos_abs[contact_index];
    // world
    state.L_contact_mea = state.L_com + com_rel_contact.cross(state.P_com);

    // kf for angular momentum

    double u_l, Rt, Qt, Kt;

    // Lx
    Qt = std::pow(dt * state.total_mass * gravity, 2) * Cov_px_;
    Rt = Cov_Lx_;

    u_l = -dt * state.total_mass * gravity * com_rel_contact[1];

    double Lx_bar = state.Lx_est + u_l;
    double sigma_Lx_bar = sigma_Lx_ + Qt;
    Kt = sigma_Lx_bar / (sigma_Lx_bar + Rt);
    state.Lx_est = Lx_bar + Kt * (state.L_contact_mea[0] - Lx_bar);
    sigma_Lx_ = (1 - Kt) * sigma_Lx_bar;

    // Ly
    Qt = std::pow(dt * state.total_mass * gravity, 2) * Cov_py_;
    Rt = Cov_Ly_;

    u_l = dt * state.total_mass * gravity * com_rel_contact[0];

    double Ly_bar = state.Ly_est + u_l;
    double sigma_Ly_bar = sigma_Ly_ + Qt;
    Kt = sigma_Ly_bar / (sigma_Ly_bar + Rt);
    state.Ly_est = Ly_bar + Kt * (state.L_contact_mea[1] - Ly_bar);
    sigma_Ly_ = (1 - Kt) * sigma_Ly_bar;

    Eigen::Vector3d L_est = {state.Lx_est, state.Ly_est, 0};

    Eigen::Vector3d L_est_f = quat_rotate_yaw_inverse(state.foot_quat_world[contact_index], L_est);

    state.Lx_est_f = L_est_f[0];
    state.Ly_est_f = L_est_f[1];

    Eigen::Vector3d com_rel_contact_f = quat_rotate_yaw_inverse(state.foot_quat_world[contact_index], com_rel_contact);

    state.px_f = com_rel_contact_f[0];
    state.py_f = com_rel_contact_f[1];
}


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

    void update_step_command();

    void load_config(std::string config_path);

    void infer_loop();
    void pd_controller_loop();
    void state_estimate_loop();
    void main_thread();
 
    void commands_callback(const communication::msg::MotionCommands::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void touch_callback(const communication::msg::TouchSensor::SharedPtr msg);
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    double smooth_sqr_wave(double sin_phase,double eps = 0.1);

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
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr myodom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;


    std::thread infer_loop_thread_;
    std::thread pd_controller_thread_;
    std::thread state_estimate_thread_;
    std::thread main_thread_;

    bool run_ = false;
    RobotState robot_state_;
    Estimator estimator_;

    // --------通过yaml初始化--------
    float pd_rate_;   //pd频率
    float infer_rate_;    //rl频率降低比例
    float estimate_rate_; // 状态观察线程降低比例
    
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
    Eigen::VectorXf torque_limits_low_;
    Eigen::VectorXf torque_limits_high_;
    std::vector<std::string> joint_names_;

   
    //Model input/output data
    Eigen::VectorXf obs_cur_; // 47
    std::deque<Eigen::VectorXf> obs_buffer_;
    Eigen::VectorXf obs_input_; // 47*3
    Eigen::VectorXf action_output_; // 12
    Eigen::VectorXf target_torques_;
    Eigen::VectorXf target_torques_prev_;


    double foot_target_theta[2];

    // clock signal
    double total_time_ = 0;
    double sin_phase_ = 0;
    double cos_phase_ = 0;
    double contact_schedule_ = 0;

    // left 0 right 1
    int contact_index_ = 0;

    // --------------------

    //Sensor data & Commands
    communication::msg::MotionCommands::SharedPtr motion_commands_msg_;
    sensor_msgs::msg::Imu::SharedPtr imu_msg_;
    sensor_msgs::msg::JointState::SharedPtr joint_state_msg_;
    communication::msg::TouchSensor::SharedPtr touch_sensor_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;
};




RobotController::RobotController() : Node("RobotController")
{

    this->declare_parameter("/topic_prefix", "hardware/");

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

    // debug publisher
    debug_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        topic_prefix_ + "robot_debug", qos);

    // action publisher
    action_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        topic_prefix_ + "action_output", qos);
    
    myodom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        topic_prefix_ + "myodom", qos);

    load_config("src/bxi_controller/config/controller.yaml");
    robot_state_.reset("src/bxi_controller/config/init_state.yaml", joint_num_);

   
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

    target_torques_.resize(action_dim_);
    target_torques_.setZero();
    target_torques_prev_.resize(action_dim_);
    target_torques_prev_.setZero();

    RCLCPP_INFO(this->get_logger(), "RobotController initialized");
}

void RobotController::load_config(std::string filename)
{
    YAML::Node config = YAML::LoadFile(filename);
    pd_rate_ = config["pd_rate"].as<double>();
    infer_rate_ = config["infer_rate"].as<float>();
    estimate_rate_ = config["estimate_rate"].as<float>();
    obs_dim_ = config["obs_dim"].as<int>();
    cycle_time_ = config["cycle_time"].as<float>();
    stack_size_ = config["stack_size"].as<int>();
    action_dim_ = config["action_dim"].as<int>();
    joint_num_ = config["joint_num"].as<int>();
    action_scales_ = config["action_scales"].as<float>();
    clip_observations_ = config["clip_observations"].as<float>();
    clip_actions_ = config["clip_actions"].as<float>();

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
    obs_scales_["height_measurements"] = config["obs_scales"]["height_measurements"].as<float>();

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
    // std::lock_guard<std::shared_mutex> lock(motion_commands_mutex_);
    motion_commands_msg_ = msg;
}

void RobotController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_msg_ = msg;

    robot_state_.imu_acc(0) = imu_msg_->linear_acceleration.x;
    robot_state_.imu_acc(1) = imu_msg_->linear_acceleration.y;
    robot_state_.imu_acc(2) = imu_msg_->linear_acceleration.z;

    robot_state_.imu_ang_vel(0) = imu_msg_->angular_velocity.x;
    robot_state_.imu_ang_vel(1) = imu_msg_->angular_velocity.y;
    robot_state_.imu_ang_vel(2) = imu_msg_->angular_velocity.z;

    robot_state_.root_quat.x() = imu_msg_->orientation.x;
    robot_state_.root_quat.y() = imu_msg_->orientation.y;
    robot_state_.root_quat.z() = imu_msg_->orientation.z;
    robot_state_.root_quat.w() = imu_msg_->orientation.w;
}

void RobotController::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_state_msg_ = msg;
    for (int i = 0; i < joint_num_; ++i)
    {
        robot_state_.joint_pos(i) = static_cast<double>(joint_state_msg_->position[i]);
        robot_state_.joint_vel(i) = static_cast<double>(joint_state_msg_->velocity[i]);
    }
}

void RobotController::touch_callback(const communication::msg::TouchSensor::SharedPtr msg)
{
    touch_sensor_msg_ = msg;
    for (int i = 0; i < NUM_LEG; i++)
    {
        robot_state_.touch_force(i) = touch_sensor_msg_->value[i];
    }
}
// only for simulation
void RobotController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
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

    if (state_estimate_thread_.joinable())
    {
        state_estimate_thread_.join();
        std::cout << "state_estimate_thread_ joined" << std::endl;
    }
}

void RobotController::state_estimate_loop()
{
    rclcpp::Rate loop_rate_(estimate_rate_);
    RCLCPP_INFO(this->get_logger(), "state_estimate_loop start !");
    // 初始化时间点
    total_time_ = 0;
    auto last_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && run_)
    {
        try
        {
            // 获取当前时间点
            auto current_time = std::chrono::steady_clock::now();
            // 计算循环的实际运行时间（单位为秒）
            std::chrono::duration<double> elapsed_time = current_time - last_time;
            // 更新上一次时间
            last_time = current_time;
            double dt = elapsed_time.count();
            total_time_ += dt;
        
            sin_phase_ = std::sin(2 * (M_PI)*total_time_ / cycle_time_);
            cos_phase_ = std::cos(2 * (M_PI)*total_time_ / cycle_time_);

            contact_schedule_ = smooth_sqr_wave(sin_phase_);

            double estimated_contacts[2];
            estimated_contacts[0] = (contact_schedule_+1.)/2.;
            estimated_contacts[1] = 1 - estimated_contacts[0];
            
            contact_index_ = (sin_phase_ >= 0) ? 0 : 1;

            estimator_.update_estimate(robot_state_, estimated_contacts, contact_index_, dt);
            
            // publish odom
            nav_msgs::msg::Odometry odom;
            odom.header.frame_id = "botelf";
            odom.header.stamp = this->now();
            odom.pose.pose.position.x = robot_state_.root_pos(0);
            odom.pose.pose.position.y = robot_state_.root_pos(1);
            odom.pose.pose.position.z = robot_state_.root_pos(2);
            odom.pose.pose.orientation.x = robot_state_.root_quat.x();
            odom.pose.pose.orientation.y = robot_state_.root_quat.y();
            odom.pose.pose.orientation.z = robot_state_.root_quat.z();
            odom.pose.pose.orientation.w = robot_state_.root_quat.w();
            odom.twist.twist.linear.x = robot_state_.world_lin_vel(0);
            odom.twist.twist.linear.y = robot_state_.world_lin_vel(1);
            odom.twist.twist.linear.z = robot_state_.world_lin_vel(2);
            myodom_pub_->publish(odom);
            loop_rate_.sleep();
        }
        catch (const std::exception &e)
        {
            RCLCPP_INFO(this->get_logger(), "state_estimate_loop error");
            RCLCPP_ERROR(this->get_logger(), e.what());
        }
    }
}

using namespace rclcpp::executors;
void RobotController::main_thread()
{
    // std::this_thread::sleep_for(5ms);
    state_estimate_thread_ = std::thread(&RobotController::state_estimate_loop, this);
}

double RobotController::smooth_sqr_wave(double sin_phase,double eps)
{
    return sin_phase / std::sqrt(sin_phase * sin_phase + eps * eps);
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

    auto robotControllerPtr = std::make_shared<RobotController>();

    try {
        robotControllerPtr->init();
        executor.add_node(robotControllerPtr);
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(robotControllerPtr->get_logger(), e.what() << '\n');
    }
    rclcpp::shutdown();
    return 0;
}
