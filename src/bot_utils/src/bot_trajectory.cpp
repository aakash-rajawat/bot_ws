#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


class BotTrajectory : public rclcpp::Node
{
public:
    BotTrajectory(const std::string& name)
        : Node(name)
    {
        constexpr int subQOS {10};
        m_odom_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/bot_controller/odom",
            subQOS,
            std::bind(&BotTrajectory::odomCallback, this, std::placeholders::_1)
        );

        constexpr int pubQOS {10};
        m_path_pub = create_publisher<nav_msgs::msg::Path>(
            "/bot_controller/trajectory",
            pubQOS
        );

        m_path.header.frame_id = "odom";
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry& message)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = message.header;
        pose.pose = message.pose.pose;

        m_path.header.stamp = message.header.stamp;
        m_path.header.frame_id = message.header.frame_id.empty() ? "odom" : message.header.frame_id;

        m_path.poses.push_back(pose);
        m_path_pub->publish(m_path);

    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub {};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub {};
    nav_msgs::msg::Path m_path {};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BotTrajectory>("bot_trajectory"));
    rclcpp::shutdown();
    return 0;
}
