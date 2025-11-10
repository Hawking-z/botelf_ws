#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <algorithm>

class GaitScheduler {
public:
    float dt;  // 时间步长
    float eps;  // 精度
    float phase;  // 连续时间相位（秒）
    float phi, phiR;  // 左右脚的归一化相位
    float fracL, fracR;  // 左右脚的子阶段进度
    float l_sin_phase, r_sin_phase;  // 左右脚的正弦相位
    float l_square_wave, r_square_wave;  // 左右脚的平滑方波
    bool l_stance_mask, r_stance_mask;  // 左右脚的支撑掩码
    float desired_contact_foot;  // 期望接触脚
    float stand_rate, full_period, delta;  // 步态周期相关的参数

    GaitScheduler(float dt = 0.01, float eps = 0.2)
        : dt(dt), eps(eps), phase(0.0), phi(0.0), phiR(0.0), 
          fracL(0.0), fracR(0.0), l_sin_phase(0.0), r_sin_phase(0.0), 
          l_square_wave(0.0), r_square_wave(0.0), 
          l_stance_mask(true), r_stance_mask(false),
          desired_contact_foot(0.0),
          stand_rate(0.5), full_period(1.0), delta(0.0) {}

    // 相位包裹到 [0, 1)
    float wrap01(float x) {
        x -= floor(x);
        return (x < 1.0) ? x : 0.0;
    }

    // 计算步态参数
    void calculate_schedule() {
        float T = full_period;
        float s = std::min(std::max(stand_rate, 1e-6f), 1.0f - 1e-6f);
        float f = 1.0f - s;

        // 归一化相位（左脚）
        phi = (T > 0) ? wrap01(phase / T) : 0.0f;
        phiR = wrap01(phi + 0.5f);
        
        // 计算左右脚的子阶段进度
        fracL = (phi < s) ? (phi / s) : ((phi - s) / f);
        fracR = (phiR < s) ? (phiR / s) : ((phiR - s) / f);

        // 计算左右脚的正弦相位
        l_sin_phase = (phi < s) ? std::sin(M_PI * fracL) : -std::sin(M_PI * fracL);
        r_sin_phase = (phiR < s) ? std::sin(M_PI * fracR) : -std::sin(M_PI * fracR);

        // 计算左右脚的平滑方波
        l_square_wave = l_sin_phase / std::sqrt(l_sin_phase * l_sin_phase + eps * eps);
        r_square_wave = r_sin_phase / std::sqrt(r_sin_phase * r_sin_phase + eps * eps);

        // 支撑掩码
        l_stance_mask = (phi < s);
        r_stance_mask = (phiR < s);

        // 期望接触脚
        float phi_shift = wrap01(phi - delta);
        if (phi_shift < 0.5f) {
            desired_contact_foot = 0.0f;  // 左脚
        } else {
            desired_contact_foot = 1.0f;  // 右脚
        }
    }

    // 更新相位
    void update_phase() {
        phase += dt;
        if (phase >= full_period) {
            phase -= full_period;
        } else if (phase < 0.0f) {
            phase += full_period;
        }
        calculate_schedule();
    }

    // 更新周期和站立比率
    void update_period(float new_full_period, float new_stand_rate) {
        full_period = new_full_period;
        stand_rate = std::min(std::max(new_stand_rate, 1e-6f), 1.0f - 1e-6f);
        delta = 0.5 * stand_rate - 0.25;
        calculate_schedule();  // 更新步态调度
    }

    // 获取所有的步态数据
    void get_gait_data(int steps, std::vector<float>& L_sin, std::vector<float>& R_sin,
                       std::vector<float>& L_sq, std::vector<float>& R_sq,
                       std::vector<bool>& L_mask, std::vector<bool>& R_mask,
                       std::vector<float>& desired) {
        for (int i = 0; i < steps; ++i) {
            L_sin.push_back(l_sin_phase);
            R_sin.push_back(r_sin_phase);
            L_sq.push_back(l_square_wave);
            R_sq.push_back(r_square_wave);
            L_mask.push_back(l_stance_mask);
            R_mask.push_back(r_stance_mask);
            desired.push_back(desired_contact_foot);
            update_phase();  // 更新相位
        }
    }

    // 保存数据到文件
    void save_to_file(const std::vector<float>& L_sin, const std::vector<float>& R_sin,
                      const std::vector<float>& L_sq, const std::vector<float>& R_sq,
                      const std::vector<bool>& L_mask, const std::vector<bool>& R_mask,
                      const std::vector<float>& desired, const std::string& filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            for (size_t i = 0; i < L_sin.size(); ++i) {
                file << i * dt << ", " << L_sin[i] << ", " << R_sin[i] << ", "
                     << L_sq[i] << ", " << R_sq[i] << ", " << L_mask[i] << ", "
                     << R_mask[i] << ", " << desired[i] << std::endl;
            }
            file.close();
        } else {
            std::cerr << "无法打开文件！" << std::endl;
        }
    }
};
