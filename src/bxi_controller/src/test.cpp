#include "utils/cfgutils.h"

#include "utils/ovutils.h"

#include <iostream>
#include <string>

int main() {
    try {
        std::string cfg_name = "/home/zyc/RoboGym/play_logs/elf1_alip_guide/service_Nov07_20-42-28_elf1_alip_guide_model_9999/exported/robot_config.yaml";
        std::string model_name = "/home/zyc/RoboGym/play_logs/elf1_alip_guide/service_Nov07_20-42-28_elf1_alip_guide_model_9999/exported/elf1_alip_guide.onnx";
        const std::string dev = "CPU";

        auto cfg = cfgutils::RobotConfig::FromYamlFile(cfg_name);
        cfgutils::ObsAssembler assembler(cfg);

        std::cout << "num_actions: " << cfg.num_actions << "\n";
        std::cout << "DOF: " << cfg.dof.isaac_order.size() << "\n";
        std::cout << "sensors: " << cfg.sensors.size() << ", obs: " << cfg.obs_map.size() << "\n";

        // 打印每个传感器与 obs 的缓冲尺寸
        for (const auto& kv : assembler.sensor_buffers()) {
            const auto& name = kv.first;
            const auto& sb   = kv.second;
            std::cout << "sensor[" << name << "]: dim=" << sb.feature_dim() << ", frames=" << sb.length() << "\n";
        }
        for (const auto& kv : assembler.obs_buffers()) {
            const auto& name = kv.first;
            const auto& ob   = kv.second;
            std::cout << "obs[" << name << "]: dim=" << ob.feature_dim()
                      << ", len=" << ob.length() << "\n";
        }

        ovutils::OVModelIO io(model_name,dev);
        io.print_summary();

        // 每步：准备当前帧的 sensor 数据（展平后！）
        std::unordered_map<std::string, Eigen::VectorXf> sensors;
        sensors["base_ang_vel"]    = Eigen::Vector3f(1,2,3);
        sensors["base_euler_xyz"]  = Eigen::Vector3f(4,5,6);
        sensors["command_lin_vel"] = Eigen::Vector2f(7,8);
        sensors["command_ang_vel"] = Eigen::VectorXf::Constant(1, 0.5f);
        sensors["dof_pos"]        = Eigen::VectorXf::LinSpaced(20, 0.0f, 1.9f);
        sensors["dof_vel"]        = Eigen::VectorXf::LinSpaced(20, 0.0f, 39.9f);
        sensors["actions"]        = Eigen::VectorXf::LinSpaced(20, 0.0f, 1.9f);
        sensors["sin_phase"]     = Eigen::VectorXf::Constant(2, 0.5f);
        // ... 把 obs 用到的所有 sensor 都填上
        auto obs_dict = assembler.step(sensors);

        for(const auto& kv : assembler.obs_buffers())
        {
            const auto& name = kv.first;

            auto& obs_data = obs_dict.at(name);
            const size_t D = static_cast<size_t>(obs_data.size());
            auto& t = io.tensor(name);
            float* p = t.data<float>();
            std::memcpy(p,obs_data.data(),sizeof(float)*D);
            // io.infer_request().set_input_tensor(name, io.tensor(name));
        }
        io.infer_request().start_async();
        io.infer_request().wait();
        auto output_tensor = io.infer_request().get_output_tensor();
        auto output_buf = output_tensor.data<const float>();
        for(int i = 0 ; i < 21; ++i)
        {
            std::cout << output_buf[i] << std::endl;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << "\n";
        return 2;
    }
}