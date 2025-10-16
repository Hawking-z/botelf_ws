#include<iostream>
#include <openvino/openvino.hpp>
#include <chrono>
void printInputAndOutputsInfo(const ov::Model& network) {
    std::cout << "model name: " << network.get_friendly_name() << std::endl;

    const std::vector<ov::Output<const ov::Node>> inputs = network.inputs();
    for (const ov::Output<const ov::Node>& input : inputs) {
        std::cout << "    inputs" << std::endl;

        const std::string name = input.get_names().empty() ? "NONE" : input.get_any_name();
        std::cout << "        input name: " << name << std::endl;

        const ov::element::Type type = input.get_element_type();
        std::cout << "        input type: " << type << std::endl;

        const ov::Shape shape = input.get_shape();
        std::cout << "        input shape: " << shape << std::endl;
    }

    const std::vector<ov::Output<const ov::Node>> outputs = network.outputs();
    for (const ov::Output<const ov::Node>& output : outputs) {
        std::cout << "    outputs" << std::endl;

        const std::string name = output.get_names().empty() ? "NONE" : output.get_any_name();
        std::cout << "        output name: " << name << std::endl;

        const ov::element::Type type = output.get_element_type();
        std::cout << "        output type: " << type << std::endl;

        const ov::Shape shape = output.get_shape();
        std::cout << "        output shape: " << shape << std::endl;
    }
}

int main()
{
    std::cout<<"Hello, world!"<<std::endl;
    std::cout<<"OpenVINO version: "<<ov::get_openvino_version()<<std::endl;
    // std::string model_path = "src/bxi_controller/models/rl_model/elf12_policy.onnx";
    std::string model_path = "src/fr_control/models/trimesh_z.onnx";
    std::cout<<"model path: "<<model_path<<std::endl;

    ov::Core core;
    std::shared_ptr<ov::Model> model = core.read_model(model_path);
    std::cout<<"model name: "<<model->get_friendly_name()<<std::endl;
    printInputAndOutputsInfo(*model);
    
    ov::CompiledModel compiled_model = core.compile_model(model_path, "CPU"); 


    ov::InferRequest infer_request = compiled_model.create_infer_request();

    std::cout<<"InferRequest created"<<std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    auto data = std::vector<float>(705, 0.0);

    for(int i =0;i<10000;i++)
    {
        auto input = compiled_model.input();
        // std::cout<<"input shape: "<<input.get_shape()<<std::endl;
        // std::cout<<"input element type: "<<input.get_element_type()<<std::endl;
        auto input_type = input.get_element_type();
        auto input_shape = input.get_shape();

        ov::Tensor input_tensor = ov::Tensor(input_type,input_shape, data.data());
        // std::cout<<"input tensor created"<<std::endl;
        infer_request.set_input_tensor(input_tensor);

        infer_request.start_async();
        infer_request.wait();

        auto output = infer_request.get_output_tensor();
        // std::cout<<"output shape: "<<output.get_shape()<<std::endl;
        // std::cout<<"output element type: "<<output.get_element_type()<<std::endl;
        // auto output_buf = output.data<const float>();
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Elapsed time: " << duration.count()/10000.0 << " microseconds" << std::endl;
    // for(int i = 0; i < 12; i++)
    // {
    //     std::cout<<output_buf[i]<<" ";
    // }
    return 0;
}