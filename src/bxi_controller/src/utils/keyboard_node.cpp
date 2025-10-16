#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>
#include <termios.h>
#include <yaml-cpp/yaml.h>
#include "communication/msg/motion_commands.hpp"

class KeyboardNode : public rclcpp::Node
{
public:
    KeyboardNode() : Node("keyboard_node"), running_(true)
    {
        motion_command_pub_ = this->create_publisher<communication::msg::MotionCommands>("motion_commands", 10);
        motion_commands_.header.frame_id = std::string("bot_elf");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&KeyboardNode::publish_motion_command, this));
        set_terminal_mode();
        load_config("src/bxi_controller/config/keyboard_config.yaml");
        input_thread_ = std::thread(&KeyboardNode::read_keyboard_input, this);
        RCLCPP_INFO(this->get_logger(), "Keyboard node started.");
    }

    ~KeyboardNode()
    {
        running_ = false;

        if (input_thread_.joinable())
        {
            input_thread_.join();
        }
        reset_terminal_mode();
        RCLCPP_INFO(this->get_logger(), "Keyboard node stopped.");
    }

private:
    void set_terminal_mode()
    {
        struct termios new_termios;
        tcgetattr(0, &old_termios);
        new_termios = old_termios;
        new_termios.c_lflag &= ~ICANON;
        new_termios.c_lflag &= ~ECHO;
        tcsetattr(0, TCSANOW, &new_termios);
    }

    void reset_terminal_mode()
    {
        tcsetattr(0, TCSANOW, &old_termios);
    }

    void load_config(std::string filename)
    {
        std::cout << "load config file: " << filename << std::endl;
        YAML::Node config = YAML::LoadFile(filename);
        max_vx = config["max_vx"].as<float>();
        max_vy = config["max_vy"].as<float>();
        max_vz = config["max_vz"].as<float>();
        max_yawdot = config["max_yawdot"].as<float>();

        min_vx = config["min_vx"].as<float>();
        min_vy = config["min_vy"].as<float>();
        min_vz = config["min_vz"].as<float>();
        min_yawdot = config["min_yawdot"].as<float>();

        vx_step = config["vx_step"].as<float>();
        vy_step = config["vy_step"].as<float>();
        vz_step = config["vz_step"].as<float>();
        yawdot_step = config["yawdot_step"].as<float>();
        RCLCPP_INFO(this->get_logger(), "Keyboard config loaded.");

    }

    void read_keyboard_input()
    {
        while (running_)
        {
            char key = get_key();
            std::lock_guard<std::mutex> lock(motion_commands_mutex_);
            switch (key)
            {
            case 'w':
                motion_commands_.vel_des.x += vx_step;
                break;
            case 's':
                motion_commands_.vel_des.x -= vx_step;
                break;
            case 'a':
                motion_commands_.vel_des.y += vy_step;
                break;
            case 'd':
                motion_commands_.vel_des.y -= vy_step;
                break;
            case 'u':
                motion_commands_.vel_des.z += vz_step;
                break;
            case 'i':
                motion_commands_.vel_des.z -= vz_step;
                break;
            case 'm':
                motion_commands_.mode = (motion_commands_.mode == 0) ? 1 : 0;
                break;
            case 'h':
                motion_commands_.height_des += 1.0;
                break;
            case 'l':
                motion_commands_.height_des -= 1.0;
                break;
            case 'q':
                motion_commands_.yawdot_des += yawdot_step;
                break;
            case 'e':
                motion_commands_.yawdot_des -= yawdot_step;
                break;
            default:
                motion_commands_.mode = 0;
                motion_commands_.vel_des.x = 0.0;
                motion_commands_.vel_des.y = 0.0;
                motion_commands_.vel_des.z = 0.0;
                motion_commands_.height_des = 0.8;
                motion_commands_.yawdot_des = 0;
                break;
            }
            motion_commands_.vel_des.x = std::clamp(motion_commands_.vel_des.x, static_cast<double>(min_vx), static_cast<double>(max_vx));
            motion_commands_.vel_des.y = std::clamp(motion_commands_.vel_des.y, static_cast<double>(min_vy), static_cast<double>(max_vy));
            motion_commands_.vel_des.z = std::clamp(motion_commands_.vel_des.z, static_cast<double>(min_vz),static_cast<double> (max_vz));
            motion_commands_.yawdot_des = std::clamp(motion_commands_.yawdot_des, min_yawdot, max_yawdot);
            RCLCPP_INFO(this->get_logger(), "key:%c vx:%f vy:%f vz:%f yawdot:%f", key, motion_commands_.vel_des.x, motion_commands_.vel_des.y, motion_commands_.vel_des.z, motion_commands_.yawdot_des);
        }
    }

    char get_key()
    {
        char buf = 0;

        if (read(0, &buf, 1) < 0)
        {
            perror("read");
        }
        return buf;
    }

    void publish_motion_command()
    {
        std::lock_guard<std::mutex> lock(motion_commands_mutex_);
        motion_commands_.header.stamp = this->now();
        motion_command_pub_->publish(motion_commands_);
    }

    rclcpp::Publisher<communication::msg::MotionCommands>::SharedPtr motion_command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;
    std::mutex motion_commands_mutex_;
    communication::msg::MotionCommands motion_commands_;
    struct termios old_termios;
    bool running_;

    float max_vx = 0;
    float max_vy = 0;
    float max_vz = 0;
    float max_yawdot = 0;

    float min_vx = 0;
    float min_vy = 0;
    float min_vz = 0;
    float min_yawdot = 0;

    float vx_step = 0;
    float vy_step = 0;
    float vz_step = 0;
    float yawdot_step = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardNode>());
    rclcpp::shutdown();
    return 0;
}