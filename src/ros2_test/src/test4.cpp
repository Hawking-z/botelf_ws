
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <iostream>
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
#include "pinocchio/algorithm/compute-all-terms.hxx"
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

class RobotState
{
public:
    void reset(std::string config_path,int dof_num)
    {
        YAML::Node config = YAML::LoadFile(config_path);
        // total_mass = config["total_mass"].as<float>();
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
        joint_pos.resize(dof_num);
        joint_vel.resize(dof_num);
        joint_pos.lazyAssign(init_dof_pos_);
        joint_vel.setZero();
        imu_acc.setZero();
        imu_ang_vel.setZero();
        root_pos = init_root_pos;
        root_quat.setIdentity();
        root_euler.setZero();
        root_rot_mat.setIdentity();

        root_lin_vel.setZero();
        // root_ang_vel.setZero();

        foot_pos_abs.setZero();
        foot_vel_abs.setZero();
        foot_quat_world.setZero();
        
    }

    // IMU sensor data
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_ang_vel;
    
    // importance kinematic variavles
    Eigen::Vector3d root_pos;
    Eigen::Vector3d root_lin_vel;
    Eigen::Quaterniond root_quat;
    // rpy
    Eigen::Vector3d root_euler;
    Eigen::Matrix3d root_rot_mat;




    Eigen::VectorXd joint_pos;
    Eigen::VectorXd joint_vel;

    // touch force
    Eigen::Vector2d touch_force;
    
    // // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs;  
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs;
    Eigen::Matrix<double, 4, NUM_LEG> foot_quat_world;


    float total_mass;
    Eigen::Vector3d com_pos;
    Eigen::Vector3d angular_momentum;

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
    void update_estimate(RobotState& state,double dt);
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

    // configuration
    Eigen::VectorXd q;
    Eigen::VectorXd v;


    double estimated_contacts[2];
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
    Eigen::Matrix3d R;                  // 新增足底坐标系的偏置旋转矩阵
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
    for(pinocchio::JointIndex jointIdx = 0; jointIdx < (pinocchio::JointIndex)model_biped.njoints; jointIdx++)
		std::cout << std::setw(10) << std::left
				  << jointIdx << ' ' << model_biped.names[jointIdx] << std::endl;

    data_biped = pinocchio::Data(model_biped);
    q.resize(model_biped.nq);
    v.resize(model_biped.nv);
    q.setZero();
    v.setZero();

    // 遍历所有 link 的惯性信息
    for (size_t i = 0; i < model_biped.inertias.size(); ++i)
    {
        const std::string& link_name = model_biped.names[i]; // 获取 link 的名称
        auto inertia = model_biped.inertias[i]; // 获取惯性信息

        // // 输出惯性信息
        std::cout << "Link: " << link_name << std::endl;
        std::cout << "  Mass: " << inertia.mass() << std::endl;
        std::cout << "  Lever (center of mass): " << inertia.lever().transpose() << std::endl;
        std::cout << "  Inertia matrix (relative to CoM):" << std::endl;
        // 输出惯性矩阵（方式 1：转换为 Eigen::Matrix3d）
        std::cout << "  Inertia matrix (dense):" << std::endl;
        std::cout << inertia.inertia().matrix() << std::endl;
    }
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
}

void Estimator::update_estimate(RobotState &state, double dt)
{
    state.root_rot_mat = state.root_quat.toRotationMatrix();
    // forward kinematics
    // root pos 和 root lin vel 为 0

    q(0) = 0.0;  q(1) = 0.0;  q(2) = 0.0;
    v(0) = 0.0;  v(1) = 0.0;  v(2) = 0.0;
    // q(0) = state.root_pos(0);
    // q(1) = state.root_pos(1);
    // q(2) = state.root_pos(2);
    // v(0) = state.root_lin_vel(0);   
    // v(1) = state.root_lin_vel(1);
    // v(2) = state.root_lin_vel(2);

    q(3) = state.root_quat.x();  
    q(4) = state.root_quat.y();  
    q(5) = state.root_quat.z();  
    q(6) = state.root_quat.w();
    // q(3) = state.root_quat.w();  
    // q(4) = state.root_quat.x();  
    // q(5) = state.root_quat.y();  
    // q(6) = state.root_quat.z();
    
    Eigen::VectorXd joint_pos_reordered(state.joint_pos.size());  // 按照新顺序重组
    Eigen::VectorXd joint_vel_reordered(state.joint_vel.size());  // 按照新顺序重组
    joint_pos_reordered << state.joint_pos.segment(0, 6), state.joint_pos.segment(12, 4), 
                            state.joint_pos.segment(6, 6), state.joint_pos.segment(16, 4);

    joint_vel_reordered << state.joint_vel.segment(0, 6), state.joint_vel.segment(12, 4), 
                            state.joint_vel.segment(6, 6), state.joint_vel.segment(16, 4);

    q.tail(model_biped.nq-7) = joint_pos_reordered;

    v.block(3,0,3,1) = state.imu_ang_vel;
    v.tail(model_biped.nv-6) = joint_vel_reordered;
    std::cout << "q: " << q.transpose() << std::endl;
    std::cout << "v: " << v.transpose() << std::endl;
    // pinocchio::forwardKinematics(model_biped, data_biped, q, v);
    // pinocchio::computeCentroidalMomentum(model_biped, data_biped, q, v);
    pinocchio::computeAllTerms(model_biped, data_biped, q, v);
    pinocchio::updateFramePlacements(model_biped, data_biped);

    // for(int i = 0;i<NUM_LEG;i++)
    // {
    //     Eigen::Vector3d fk_pos = data_biped.oMf[foot_end_idx[i]].translation();
    //     Eigen::Vector3d leg_v = pinocchio::getFrameVelocity(model_biped, data_biped, foot_end_idx[i], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED).linear();
    //     Eigen::Matrix3d  fk_quat = data_biped.oMf[foot_end_idx[i]].rotation();
    //     std::cout << "fk_pos: " << fk_pos.transpose() << std::endl;
    //     std::cout << "leg_v: " << leg_v.transpose() << std::endl;
    //     std::cout << "fk_quat: " << fk_quat << std::endl;
    //     state.foot_pos_abs.col(i) = fk_pos;
    //     state.foot_vel_abs.col(i) = leg_v;
    // }

    double mass = data_biped.Ig.mass();



    Eigen::Vector3d linear_momentum = data_biped.hg.linear();
    Eigen::Vector3d angular_momentum = data_biped.hg.angular();

    std::cout<< "mass: " << data_biped.Ig.mass() << std::endl;
  
    std::cout<<"com: "<<data_biped.com[0]<<std::endl;
    std::cout<<"com_vel: "<<data_biped.vcom[0]<<std::endl;
    // std::cout<<data_biped.vcom<<std::endl;
    std::cout << "linear_momentum: " << linear_momentum.transpose() << std::endl;
    std::cout << "angular_momentum: " << angular_momentum.transpose() << std::endl;


    std::cout<<"--------------------------- "<<std::endl;
    std::cout<<"num of link: "<<model_biped.njoints<<std::endl;
    Eigen::Vector3d world_root_lin_vel = state.root_rot_mat * state.root_lin_vel;
    // Eigen::Vector3d world_root_ang_vel = state.root_rot_mat * state.imu_ang_vel;
    Eigen::Vector3d world_com_pos =  data_biped.com[0] + state.root_pos;
    Eigen::Vector3d world_com_vel = data_biped.vcom[0] + world_root_lin_vel;
    Eigen::Vector3d world_linear_momentum = mass*world_com_vel;
    Eigen::Vector3d world_angular_momentum = angular_momentum ;

    std::cout << "world_com_pos: " << world_com_pos.transpose() << std::endl; 
    std::cout << "world_com_vel: " << world_com_vel.transpose() << std::endl;
    std::cout << "world_linear_momentum: " << world_linear_momentum.transpose() << std::endl;
    std::cout << "world_angular_momentum: " << world_angular_momentum.transpose() << std::endl;
    std::cout<<"--------------------------- "<<std::endl;
    q.head(3) = state.root_pos;
    v.head(3) = state.root_lin_vel;
    std::cout << "q: " << q.transpose() << std::endl;
    std::cout << "v: " << v.transpose() << std::endl;

    pinocchio::computeAllTerms(model_biped, data_biped, q, v);
    pinocchio::updateFramePlacements(model_biped, data_biped);


    Eigen::Vector3d real_linear_momentum = data_biped.hg.linear();
    Eigen::Vector3d real_angular_momentum = data_biped.hg.angular();

    std::cout<< "mass: " << data_biped.Ig.mass() << std::endl;
  
    std::cout<<"com: "<<data_biped.com[0]<<std::endl;
    std::cout<<"com_vel: "<<data_biped.vcom[0]<<std::endl;
    // std::cout<<data_biped.vcom<<std::endl;
    std::cout << "linear_momentum: " << real_linear_momentum.transpose() << std::endl;
    std::cout << "angular_momentum: " << real_angular_momentum.transpose() << std::endl;


    // update A B using latest dt
    A.block<3, 3>(0, 3) = dt * identity_3x3;
    B.block<3, 3>(0, 0) = 0.5 * dt * dt * identity_3x3;
    B.block<3, 3>(3, 0) = dt * identity_3x3;

    

    // control input
    Eigen::Vector3d u = state.root_rot_mat *state.imu_acc + Eigen::Vector3d(0, 0, -9.81);
    
    
    for(int i = 0; i < NUM_LEG; i++)
    {
        estimated_contacts[i] = std::min(std::max((state.touch_force(i)) / (150.0 - 0.0), 0.0), 1.0);
    }

    // update Q
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * identity_3x3;
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.81 / 20.0 * identity_3x3;

    // update Q R for legs not in contact
    for (int i = 0; i < NUM_LEG; ++i) {
        Q.block<3, 3>(6 + i * 3, 6 + i * 3)
                = (1 + (1 - estimated_contacts[i]) * MAX_NUM) * dt * PROCESS_NOISE_PFOOT * identity_3x3;  // foot position transition
        
        // for estimated_contacts[i] == 1, Q = 0.002
        // for estimated_contacts[i] == 0, Q = 1001*Q

        R.block<3, 3>(i * 3, i * 3)
                = (1 + (1 - estimated_contacts[i]) * MAX_NUM) * SENSOR_NOISE_PIMU_REL_FOOT *
                  identity_3x3;                       
        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3)
                = (1 + (1 - estimated_contacts[i]) * MAX_NUM) * SENSOR_NOISE_VIMU_REL_FOOT * identity_3x3;      // vel estimation
        R(NUM_LEG * 6 + i, NUM_LEG * 6 + i)
                = (1 + (1 - estimated_contacts[i]) * MAX_NUM) * SENSOR_NOISE_ZFOOT;       // height z estimation
    }

    // process update
    xbar = A*x + B*u;
    Pbar = A*P*A.transpose() + Q;

    yhat = C*xbar;

    for (int i=0; i<NUM_LEG; ++i) {
        Eigen::Vector3d fk_pos = state.foot_pos_abs.block<3,1>(0,i);
        y.block<3,1>(i*3,0) = -fk_pos;   // fk estimation
        y.block<3,1>(NUM_LEG*3+i*3,0) =
                (1.0-estimated_contacts[i])*x.segment<3>(3) -  estimated_contacts[i]*state.foot_vel_abs.col(i);      // vel estimation
        y(NUM_LEG*6+i) = (1.0-estimated_contacts[i])*(x(2)+fk_pos(2));                               // height z estimation
    }

    S = C * Pbar *C.transpose() + R;
    S = 0.5*(S+S.transpose());

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
    state.root_lin_vel = x.segment<3>(3);

}

/*
base_pos: tensor([[-0.0492,  0.0124,  0.8982]], device='cuda:0')
base_quat: tensor([[-0.0578, -0.0772,  0.0035,  0.9953]], device='cuda:0')
dof_vel: tensor([[ 2.0728,  2.4576,  2.2580, -0.1464,  1.5059,  1.3388,  1.5004,  0.8971,
          3.8235,  4.0451,  1.0880,  3.6087, -0.8150,  8.9512,  1.9319,  2.7752,
          0.9662,  0.6939,  1.2255,  2.3923]], device='cuda:0')
dof_pos: tensor([[ 0.1705,  0.3305, -0.1275,  1.0376, -0.1762,  0.2315,  0.6411,  0.2484,
          0.2040, -1.0707,  0.2075,  0.3473, -0.2017,  0.9435, -0.2889,  0.3391,
          0.6135, -0.0321,  0.0727, -1.0363]], device='cuda:0')
base_lin_vel: tensor([[-0.1997,  0.1249, -1.2010]], device='cuda:0')
base_ang_vel: tensor([[-0.7251, -1.2942, -0.7007]], device='cuda:0')
*/

int main()
{
    Estimator estimator;
    RobotState state;
    state.reset("/home/zyc/bxi_ws/src/bxi_controller/config/init_state.yaml", 20);
    estimator.init_state(state);
  
    state.root_quat.x() = -0.0578;
    state.root_quat.y() = -0.0772;
    state.root_quat.z() = 0.0035;
    state.root_quat.w() = 0.9953;
    state.root_quat.normalize();
    // state.root_quat.setIdentity();

    state.joint_vel << 2.0728, 2.4576, 2.2580, -0.1464, 1.5059, 1.3388,
                        1.0880, 3.6087,-0.8150, 8.9512, 1.9319, 2.7752,
                        1.5004, 0.8971, 3.8235, 4.0451, 
                        
                        0.9662, 0.6939, 1.2255, 2.3923;
    state.joint_pos << 0.1705, 0.3305, -0.1275, 1.0376, -0.1762, 0.2315,
                        0.2075, 0.3473,-0.2017, 0.9435, -0.2889, 0.3391,
                        0.6411, 0.2484, 0.2040, -1.0707, 
                        0.6135, -0.0321, 0.0727, -1.0363;
    
    state.imu_ang_vel << -0.7251, -1.2942, -0.7007;
    // state.imu_ang_vel << -0.6111, -1.3773, -0.6491;
    state.touch_force << 0, 0;
    state.root_pos << -0.0492, 0.0124, 0.8982;
    
    state.root_lin_vel << -0.1997, 0.1249, -1.2010;
    // state.root_lin_vel << -0.0121,-0.0166, -1.2237,
    
    
    estimator.update_estimate(state, 0.01);


    return 0;
}