#include <rclcpp/rclcpp.hpp>

#include"std_msgs/msg/float32_multi_array.hpp"
#include <ctime>

using namespace std::chrono_literals;

class FloatArrayPublisher : public rclcpp::Node
{
public:
    FloatArrayPublisher() : Node("float_array_publisher")
    {   
        std::srand(std::time(nullptr));
        // 创建发布者，主题类型为 Float32MultiArray
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("float_array_topic", 10);

        // 定时器，定期发布消息
        timer_ = this->create_wall_timer(500ms, std::bind(&FloatArrayPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        // 创建消息并设置值
        auto message = std_msgs::msg::Float32MultiArray();
        // message.data = {1.1, 2.2, 3.3, 4.4, 5.5};  // 设置 float32[] 数据
        for(int i = 0 ;i<5;i++)
        {
            message.data.emplace_back(rand()%100);
        }

        // 发布消息
        RCLCPP_INFO(this->get_logger(), "Publishing: [%.2f, %.2f, %.2f, %.2f, %.2f]",
                    message.data[0], message.data[1], message.data[2], message.data[3], message.data[4]);
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloatArrayPublisher>());
    rclcpp::shutdown();
    return 0;
}