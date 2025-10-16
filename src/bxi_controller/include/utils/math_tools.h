#pragma once

#include <eigen3/Eigen/Dense>

Eigen::Vector3d quat_rotate(const Eigen::Quaterniond &quat, const Eigen::Vector3d &point);

Eigen::Vector3d quat_rotate_yaw(const Eigen::Quaterniond &quat, const Eigen::Vector3d &vec);

Eigen::Vector3d quat_rotate_inverse(const Eigen::Quaterniond &quat, const Eigen::Vector3d &vec);

Eigen::Vector3d quat_rotate_yaw_inverse(const Eigen::Quaterniond &quat, const Eigen::Vector3d &vec);

Eigen::Vector3d get_euler_xyz(const Eigen::Quaterniond &quat);

double get_yaw_from_quaternion(const Eigen::Quaterniond &quat);

Eigen::Vector3d wrap_to_pi(const Eigen::Vector3d &angles);

double get_yaw_from_rotation_matrix(const Eigen::Matrix3d &rotation_matrix);

double wrap_to_pi(double angle);

// 模板低通滤波器类
template <typename T>
class LowPassFilter {
public:
    // 默认构造函数：先构造对象，不初始化平滑因子和初始输出
    LowPassFilter() : alpha_(0.0), initialized_(false) {}

    // 构造函数，传入滤波平滑因子 alpha（0~1 之间）
    explicit LowPassFilter(double alpha) : alpha_(alpha), initialized_(false) {}

    // 初始化函数：设置平滑因子和初始输出值（必须在滤波之前调用）
    void init(double alpha, const T &initialValue) {
        alpha_ = alpha;
        output_ = initialValue;
        initialized_ = true;
    }

    // 滤波函数，输入当前数据，返回滤波后的输出
    T filter(const T &input) {
        if (!initialized_) {
            // 如果未初始化，则使用第一次输入数据作为初始输出
            output_ = input;
            initialized_ = true;
        } else {
            // 根据公式更新滤波器输出
            output_ = alpha_ * input + (1.0 - alpha_) * output_;
        }
        return output_;
    }

private:
    double alpha_;   // 平滑因子
    T output_;       // 上一次滤波输出
    bool initialized_; // 是否已初始化
};
