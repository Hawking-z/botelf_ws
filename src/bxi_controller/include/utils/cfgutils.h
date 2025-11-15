#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
#include <string>
#include <numeric>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <limits>
#include <unordered_set>

namespace cfgutils {


struct DofParam {
    double default_kp = 0.0;
    double default_kd = 0.0;
    double pos =0.0;
    double vel = 0.0;
    double init_pos   = 0.0;
    uint8_t control_mode = 0;
};

inline int product(const std::vector<int>& v) {
    if (v.empty()) return 0; // 空 shape 非法
    long long p = 1;
    for (int x : v) {
        if (x <= 0) throw std::runtime_error("shape 维度必须为正整数");
        p *= x;
        if (p > std::numeric_limits<int>::max()) {
            throw std::runtime_error("shape 维度乘积过大");
        }
    }
    return static_cast<int>(p);
}

inline std::vector<int> as_vec_i(const YAML::Node& node) {
    if (!node || !node.IsSequence()) throw std::runtime_error("期望一个整数数组 (YAML sequence)");
    std::vector<int> out; out.reserve(node.size());
    for (auto it : node) out.push_back(it.as<int>());
    return out;
}

inline std::vector<std::string> as_vec_s(const YAML::Node& node) {
    if (!node || !node.IsSequence()) throw std::runtime_error("期望一个字符串数组 (YAML sequence)");
    std::vector<std::string> out; out.reserve(node.size());
    for (auto it : node) out.push_back(it.as<std::string>());
    return out;
}

inline bool node_has(const YAML::Node& n, const char* key) {
    return n[key] && !n[key].IsNull();
}

// ---------- DOF 配置 ----------
struct DofConfig {
    std::vector<std::string> isaac_order;                 // 关节顺序
    std::unordered_map<std::string, int> name_to_index;    // 名字到序号的映射
    Eigen::VectorXf kp, kd, scale, torque_limits, default_pos; // 长度 = DOF
};

// ---------- 传感器规格 ----------
struct SensorSpec {
    std::string name;
    std::vector<int> shape; // 可多维
    int elem_dim = 0;       // 展平长度 (C-order)
    int frames   = 1;       // 传感器级时间窗
    float scale  = 1.0f;    // 导入缩放
};

// ---------- Obs 规格 ----------
struct ObsSpec {
    struct SourceSlice { int frames = 1; int elem_dim = 0; int offset = 0; };

    std::string name;
    std::vector<std::string> sources; // 使用顺序
    int history_len = 1;
    bool flatten    = true;

    int per_step_dim = 0;  // Σ frames(s)*elem_dim(s)
    int final_dim    = 0;  // history_len*per_step_dim（仅当 flatten=true 时有意义）

    std::vector<SourceSlice> slices; // 与 sources 对齐，便于偏移寻址
};

// ---------- RobotConfig 汇总 ----------
struct RobotConfig {
    int num_actions = 0;
    DofConfig dof;

    std::unordered_map<std::string, SensorSpec> sensors; // name -> spec
    std::unordered_map<std::string, ObsSpec>     obs_map; // obs_name -> spec

    float clip_obs    = 0.0f;   // 可选
    float clip_actions= 0.0f;   // 可选
    float pd_rate = 1000;
    float infer_rate = 100;
    // schedule
    float step_period = 0.32;
    float stand_rate = 0.55;

    // 读取 + 校验
    static RobotConfig FromYamlFile(const std::string& path) {
        YAML::Node root = YAML::LoadFile(path);
        if (!root) throw std::runtime_error("无法加载 YAML 文件: " + path);

        RobotConfig cfg;
       
        cfg.num_actions = root["num_actions"].as<int>();
        cfg.clip_obs = root["clip_obs"].as<float>();
        cfg.clip_actions = root["clip_actions"].as<float>();
        cfg.pd_rate = root["pd_rate"].as<float>();
        cfg.infer_rate = root["infer_rate"].as<float>();
        cfg.step_period = root["step_period"].as<float>();
        cfg.stand_rate = root["stand_rate"].as<float>();

        // ---------- dof_config ----------
        if (!node_has(root, "dof_config")) throw std::runtime_error("缺少 dof_config 段");
        const auto& dcfg = root["dof_config"];
        if (!node_has(dcfg, "isaac_order")) throw std::runtime_error("dof_config 缺少 isaac_order");
        cfg.dof.isaac_order = as_vec_s(dcfg["isaac_order"]);
        for (int i = 0; i < static_cast<int>(cfg.dof.isaac_order.size()); ++i) {
            cfg.dof.name_to_index[cfg.dof.isaac_order[i]] = i;
        }
        const int DOF = static_cast<int>(cfg.dof.isaac_order.size());
        if (DOF <= 0) throw std::runtime_error("isaac_order 为空");

        auto init_vec = [&](Eigen::VectorXf& v){ v.resize(DOF); v.setZero(); };
        init_vec(cfg.dof.kp); init_vec(cfg.dof.kd); init_vec(cfg.dof.scale);
        init_vec(cfg.dof.torque_limits); init_vec(cfg.dof.default_pos);

        for (int i = 0; i < DOF; ++i) {
            const std::string& jn = cfg.dof.isaac_order[i];
            if (!node_has(dcfg, jn.c_str())) {
                throw std::runtime_error("dof_config 缺少关节参数: " + jn);
            }
            const auto& jn_node = dcfg[jn.c_str()];
            try {
                cfg.dof.kp[i]           = jn_node["kp"].as<float>();
                cfg.dof.kd[i]           = jn_node["kd"].as<float>();
                cfg.dof.scale[i]        = jn_node["action_scale"].as<float>();
                cfg.dof.torque_limits[i]= jn_node["torque_limits"].as<float>();
                cfg.dof.default_pos[i]  = jn_node["default_pos"].as<float>();
            } catch (const std::exception& e) {
                std::ostringstream oss; oss << "解析关节参数失败: " << jn << ", 错误: " << e.what();
                throw std::runtime_error(oss.str());
            }
        }

        if (!node_has(root, "obs_config")) throw std::runtime_error("缺少 obs_config 段");
        const auto& ocfg = root["obs_config"];
        if (!node_has(ocfg, "sensors")) throw std::runtime_error("obs_config 缺少 sensors 段");

        const auto& sc = ocfg["sensors"];
        if (!sc.IsMap()) throw std::runtime_error("sensors 必须是一个 map");
        for (auto it : sc) {
            const std::string name = it.first.as<std::string>();
            const YAML::Node& sn   = it.second;
            SensorSpec ss; ss.name = name;
            if (!node_has(sn, "shape")) throw std::runtime_error("sensor " + name + " 缺少 shape");
            ss.shape    = as_vec_i(sn["shape"]);
            ss.elem_dim = product(ss.shape);
            ss.frames   = node_has(sn, "frames") ? sn["frames"].as<int>() : 1;
            ss.scale    = node_has(sn, "obs_scale")  ? sn["obs_scale"].as<float>() : 1.0f;
            if (ss.frames <= 0) throw std::runtime_error("sensor " + name + " 的 frames 必须 >=1");
            cfg.sensors.emplace(name, std::move(ss));
        }
        for (auto it : ocfg) {
            const std::string key = it.first.as<std::string>();
            if (key == "sensors") continue;
            const YAML::Node& on = it.second;

            ObsSpec os; os.name = key;
            if (!node_has(on, "sources")) throw std::runtime_error("obs " + key + " 缺少 sources");
            os.sources     = as_vec_s(on["sources"]);
            os.history_len = node_has(on, "history_len") ? on["history_len"].as<int>() : 1;
            os.flatten     = node_has(on, "flatten") ? on["flatten"].as<bool>() : true;
            if (os.history_len <= 0) throw std::runtime_error("obs " + key + " 的 history_len 必须 >=1");

            int offset = 0;
            os.slices.reserve(os.sources.size());
            for (const auto& src : os.sources) {
                auto itS = cfg.sensors.find(src);
                if (itS == cfg.sensors.end()) throw std::runtime_error("obs " + key + " 引用了未知 sensor: " + src);
                const auto& ss = itS->second;
                ObsSpec::SourceSlice sl; sl.frames = ss.frames; sl.elem_dim = ss.elem_dim; sl.offset = offset;
                os.slices.push_back(sl);
                offset += sl.frames * sl.elem_dim;
            }
            os.per_step_dim = offset;
            os.final_dim    = os.history_len * os.per_step_dim;

            cfg.obs_map.emplace(os.name, std::move(os));
        }

        return cfg;
    }
};



class RingBuffer {
public:
    RingBuffer() = default;
    RingBuffer(int feature_dim, int length)
    : feat_dim_(feature_dim), len_(length), data_(feature_dim, length), head_(length - 1), steps_written_(0) {
        if (feat_dim_ <= 0 || len_ <= 0) throw std::runtime_error("RingBuffer 参数非法");
        data_.setZero();
    }

    void reset() {
        data_.setZero();
        head_ = len_ - 1; // 第一次 push 后 head_ == 0
        steps_written_ = 0;
    }

    int feature_dim() const { return feat_dim_; }
    int length()     const { return len_; }

    const Eigen::MatrixXf& raw() const { return data_; }

    int head() const { return head_; }

    void push(const Eigen::Ref<const Eigen::VectorXf>& col) {
        if (col.size() != feat_dim_) throw std::runtime_error("RingBuffer::push 维度不匹配");
        head_ = (head_ + 1) % len_;
        data_.col(head_) = col;
        ++steps_written_;
    }

    void copy_relative_col_to(int offset_back, Eigen::Ref<Eigen::VectorXf> out, int offset) const {
        if (offset < 0 || offset + feat_dim_ > out.size()) {
            throw std::runtime_error("copy_relative_col_to: 输出越界");
        }
        if (offset_back < 0) {
            throw std::runtime_error("copy_relative_col_to: offset_back < 0");
        }
        if (offset_back >= len_ || steps_written_ <= offset_back) {
            out.segment(offset, feat_dim_).setZero();
            return;
        }
        int idx = head_ - offset_back;
        if (idx < 0) idx += len_ * ((-idx) / len_ + 1);
        idx %= len_;
        out.segment(offset, feat_dim_) = data_.col(idx);
    }
    void flatten_old_to_new(Eigen::Ref<Eigen::VectorXf> out) const {
        if (out.size() != len_ * feat_dim_) throw std::runtime_error("flatten_old_to_new: 维度不匹配");
        for (int k = 0; k < len_; ++k) {
            int offset_back = len_ - 1 - k; // k=0 拿最旧, k=len-1 拿最新
            if (steps_written_ <= offset_back) {
                out.segment(k * feat_dim_, feat_dim_).setZero();
                continue;
            }
            int idx = head_ - offset_back;
            if (idx < 0) idx += len_ * ((-idx) / len_ + 1);
            idx %= len_;
            out.segment(k * feat_dim_, feat_dim_) = data_.col(idx);
        }
    }

    int steps_written() const { return steps_written_; }

private:
    int feat_dim_ = 0;   
    int len_ = 0;          
    Eigen::MatrixXf data_;   
    int head_ = 0;           
    int steps_written_ = 0;  
};

class ObsAssembler {
public:
    explicit ObsAssembler(RobotConfig cfg) : cfg_(std::move(cfg)) {
        for (const auto& kv : cfg_.obs_map) {
            for (const auto& s : kv.second.sources) required_sensors_.insert(s);
        }
        for (const auto& kv : cfg_.sensors) {
            const auto& ss = kv.second;
            sensor_input_.emplace(kv.first, Eigen::VectorXf::Zero(ss.elem_dim));
            sensor_bufs_.emplace(kv.first, RingBuffer(ss.elem_dim, ss.frames));
        }
        for (const auto& kv : cfg_.obs_map) {
            const auto& os = kv.second;
            obs_bufs_.emplace(kv.first, RingBuffer(os.per_step_dim, os.history_len));
        }
    }

    void reset_all() {
        for (auto& kv : sensor_bufs_) kv.second.reset();
        for (auto& kv : obs_bufs_)    kv.second.reset();
    }

    const RobotConfig& config() const { return cfg_; }

    const std::unordered_map<std::string, RingBuffer>& sensor_buffers() const { return sensor_bufs_; }
    const std::unordered_map<std::string, RingBuffer>& obs_buffers()    const { return obs_bufs_; }
    const std::unordered_map<std::string, Eigen::VectorXf>& sensor_input() const { return sensor_input_; }

    std::unordered_map<std::string, Eigen::VectorXf>
    step(const std::unordered_map<std::string, Eigen::VectorXf>& sensor_dict) {
        for (const auto& sname : required_sensors_) {
            auto it = sensor_dict.find(sname);
            if (it == sensor_dict.end()) {
                throw std::runtime_error(std::string("step() 缺少必需传感器输入: ") + sname);
            }
            const auto& spec = cfg_.sensors.at(sname);
            const auto& v = it->second;
            if (v.size() != spec.elem_dim) {
                std::ostringstream oss; oss << "传感器 " << sname << " 维度不匹配: got " << v.size() << ", expect " << spec.elem_dim;
                throw std::runtime_error(oss.str());
            }
            Eigen::VectorXf tmp = v * spec.scale;
            if (cfg_.clip_obs > 0.0f) {
                for (int i = 0; i < tmp.size(); ++i) {
                    if (tmp[i] >  cfg_.clip_obs) tmp[i] =  cfg_.clip_obs;
                    if (tmp[i] < -cfg_.clip_obs) tmp[i] = -cfg_.clip_obs;
                }
            }
            sensor_bufs_.at(sname).push(tmp);
        }
        for (const auto& kv : cfg_.obs_map) {
            const auto& oname = kv.first;
            const auto& ospec = kv.second;
            Eigen::VectorXf per_step(ospec.per_step_dim);
            int offset = 0;
            for (size_t i = 0; i < ospec.sources.size(); ++i) {
                const auto& sname = ospec.sources[i];
                const auto& ss    = cfg_.sensors.at(sname);
                const auto& sbuf  = sensor_bufs_.at(sname);
                for (int f = 0; f < ss.frames; ++f) {
                    int offset_back = ss.frames - 1 - f; // 旧->新
                    sbuf.copy_relative_col_to(offset_back, per_step, offset);
                    offset += ss.elem_dim;
                }
            }
            obs_bufs_.at(oname).push(per_step);
        }
        std::unordered_map<std::string, Eigen::VectorXf> out;
        out.reserve(cfg_.obs_map.size());
        for (const auto& kv : cfg_.obs_map) {
            const auto& oname = kv.first;
            const auto& ospec = kv.second;
            Eigen::VectorXf flat(ospec.history_len * ospec.per_step_dim);
            obs_bufs_.at(oname).flatten_old_to_new(flat);
            out.emplace(oname, flat);
        }
        return out;
    }
    void print_obs_dict(const std::unordered_map<std::string, Eigen::VectorXf>& obs_dict) const {
        for (const auto& kv : obs_dict) {
            const auto& name = kv.first;
            const auto& v    = kv.second;
            std::cout << "obs[" << name << "]: size=" << v.size() << ", data=[";
            for (int i = 0; i < std::min(static_cast<int>(v.size()), 10); ++i) {
                if (i) std::cout << ", ";
                std::cout << v[i];
            }
            if (v.size() > 10) std::cout << ", ...";
            std::cout << "]\n";
        }
    }

private:
    RobotConfig cfg_;
    std::unordered_map<std::string, Eigen::VectorXf> sensor_input_;
    std::unordered_map<std::string, RingBuffer> sensor_bufs_;
    std::unordered_map<std::string, RingBuffer> obs_bufs_;
    std::unordered_set<std::string> required_sensors_;
};
} 
