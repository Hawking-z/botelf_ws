
#include "utils/math_tools.h"

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

Eigen::Vector3f get_euler_xyz(const Eigen::Quaternionf &quat)
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

    // 返回 Eigen::Vector3f
    return Eigen::Vector3f(roll, pitch, yaw);
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

double get_yaw_from_rotation_matrix(const Eigen::Matrix3d &rotation_matrix)
{
    // 提取旋转矩阵中的元素
    double R10 = rotation_matrix(1, 0);
    double R00 = rotation_matrix(0, 0);
    
    // 使用atan2计算偏航角
    double yaw = std::atan2(R10, R00);

    // 归一化到 [-π, π]
    return std::remainder(yaw, 2.0 * M_PI);
}