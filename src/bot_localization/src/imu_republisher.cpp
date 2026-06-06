#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>


rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub {};

void imuCallback(const sensor_msgs::msg::Imu& imu_msg)
{
    sensor_msgs::msg::Imu new_imu {};
    new_imu = imu_msg;
    new_imu.header.frame_id = "imu_link_ekf";
    imu_pub->publish(new_imu);
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("imu_republisher_node");
    rclcpp::sleep_for(std::chrono::seconds(1));

    imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(
        "imu_ekf",
        10
    );

    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "imu/out",
        10,
        imuCallback
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
