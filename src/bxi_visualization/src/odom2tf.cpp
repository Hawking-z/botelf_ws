#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomToTF : public rclcpp::Node {
public:
    OdomToTF()
    : Node("odom_to_tf") {
        // 创建 TF 广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        // 订阅 /odom 消息
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos,
            std::bind(&OdomToTF::odomCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "odom_to_tf node started, listening to /odom");
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 创建 TransformStamped 消息
        geometry_msgs::msg::TransformStamped transform;

        // 设置时间戳和坐标系
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = "dummy_link";       // 父坐标系
        transform.child_frame_id = "base_link";  // 子坐标系

        // 设置平移
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;

        // 设置旋转（四元数）
        transform.transform.rotation = msg->pose.pose.orientation;

        // 广播变换
        tf_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToTF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}