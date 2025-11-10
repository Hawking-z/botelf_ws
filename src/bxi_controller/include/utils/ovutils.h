#pragma once

// ovutils.cpp
#include <openvino/openvino.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <limits>
#include <cstring>   // std::memset

namespace ovutils {
struct InputInfo {
    std::string       name;       // 输入名
    ov::element::Type dtype;      // 数据类型
    ov::PartialShape  pshape;     // 模型原始（可能动态）的形状
    ov::Shape         concrete;   // 实际用于分配张量的形状（动态维默认置 1）
    ov::Tensor        tensor;     // 预分配张量（已清零）
};

class OVModelIO {
public:
    explicit OVModelIO(const std::string& onnx_path, const std::string& device = "CPU") {
        // 1) 读模型（支持 ONNX）
        model_ = core_.read_model(onnx_path);

        // 2) 收集输入基本信息
        collect_inputs_();

        // 3) 编译模型 + 创建 InferRequest
        compiled_ = core_.compile_model(model_, device);
        infer_request_ = compiled_.create_infer_request();

        // 4) 为每个输入分配张量并绑定
        allocate_and_bind_();
    }

    // 覆盖某个输入的实际形状（例如把 {1,1} 改为 {1,224,224,3}）
    void set_input_shape_override(const std::string& input_name, const ov::Shape& shape) {
        auto it = std::find_if(inputs_.begin(), inputs_.end(),
                               [&](const InputInfo& x){ return x.name == input_name; });
        if (it == inputs_.end()) {
            throw std::runtime_error("No such input: " + input_name);
        }
        it->concrete = shape;
        it->tensor = ov::Tensor(it->dtype, it->concrete);
        std::memset(it->tensor.data(), 0, it->tensor.get_byte_size());
        infer_request_.set_tensor(it->name, it->tensor);
    }

    // 打印输入摘要
    void print_summary(std::ostream& os = std::cout) const {
        os << "=== Model Inputs (" << inputs_.size() << ") ===\n";
        for (const auto& in : inputs_) {
            os << "name: " << in.name
               << " | dtype: " << in.dtype.get_type_name()
               << " | partial_shape: " << partial_shape_to_string_(in.pshape)
               << " | concrete_shape: " << shape_to_string_(in.concrete)
               << "\n";
        }
    }

    // 获取某输入的可写张量（可直接填充数据）
    ov::Tensor& tensor(const std::string& input_name) {
        auto it = std::find_if(inputs_.begin(), inputs_.end(),
                               [&](const InputInfo& x){ return x.name == input_name; });
        if (it == inputs_.end()) {
            throw std::runtime_error("No such input: " + input_name);
        }
        return it->tensor;
    }

    // 访问器
    const std::vector<InputInfo>& inputs() const { return inputs_; }
    ov::InferRequest& infer_request() { return infer_request_; }
    ov::CompiledModel& compiled_model() { return compiled_; }

private:
    ov::Core core_;
    std::shared_ptr<ov::Model> model_;
    ov::CompiledModel compiled_;
    ov::InferRequest infer_request_;
    std::vector<InputInfo> inputs_;

    // —— 工具函数（类内定义，避免“extra qualification”错误）——
    static std::string dim_to_string_(const ov::Dimension& d) {
        if (d.is_dynamic()) return "?";
        const auto lo = d.get_min_length();
        const auto hi = d.get_max_length();
        if (lo == hi) {
            return std::to_string(static_cast<size_t>(lo));
        }
        std::ostringstream oss;
        oss << "[" << static_cast<size_t>(lo) << ",";
        if (hi == std::numeric_limits<ov::Dimension::value_type>::max()) {
            oss << "inf";
        } else {
            oss << static_cast<size_t>(hi);
        }
        oss << "]";
        return oss.str();
    }

    static std::string partial_shape_to_string_(const ov::PartialShape& ps) {
        if (!ps.rank().is_static()) return "{?}";
        std::ostringstream oss;
        oss << "{";
        bool first = true;
        for (const auto& d : ps) {
            if (!first) oss << ", ";
            first = false;
            oss << dim_to_string_(d);
        }
        oss << "}";
        return oss.str();
    }

    static std::string shape_to_string_(const ov::Shape& s) {
        std::ostringstream oss;
        oss << "{";
        for (size_t i = 0; i < s.size(); ++i) {
            if (i) oss << ", ";
            oss << s[i];
        }
        oss << "}";
        return oss.str();
    }

    static ov::Shape choose_concrete_shape_(const ov::PartialShape& ps) {
        ov::Shape out;
        if (ps.rank().is_static()) {
            out.reserve(static_cast<size_t>(ps.rank().get_length()));
        }
        for (const auto& d : ps) {
            if (d.is_static()) {
                out.push_back(static_cast<size_t>(d.get_length()));
            } else {
                out.push_back(1); // 动态维默认 1
            }
        }
        if (out.empty()) out = {1}; // 完全未知 rank 时给最小占位
        return out;
    }

    static std::string safe_input_name_(const ov::Output<const ov::Node>& port) {
        std::string name = port.get_any_name();
        if (!name.empty()) return name;
        // 回退：节点友好名 + 端口索引
        name = port.get_node()->get_friendly_name();
        name += ":" + std::to_string(port.get_index());
        return name;
    }

    void collect_inputs_() {
        inputs_.clear();
        for (const auto& port : model_->inputs()) {
            InputInfo info;
            info.name     = safe_input_name_(port);
            info.dtype    = port.get_element_type();
            info.pshape   = port.get_partial_shape();
            info.concrete = choose_concrete_shape_(info.pshape);
            inputs_.push_back(std::move(info));
        }
    }

    void allocate_and_bind_() {
        for (auto& in : inputs_) {
            in.tensor = ov::Tensor(in.dtype, in.concrete);
            std::memset(in.tensor.data(), 0, in.tensor.get_byte_size());
            infer_request_.set_tensor(in.name, in.tensor);
        }
    }
};
}
