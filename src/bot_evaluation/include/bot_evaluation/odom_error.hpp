#ifndef ODOM_ERROR_HPP
#define ODOM_ERROR_HPP

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

class OdomError : public rclcpp::Node
{
public:
    explicit OdomError(const std::string& name);

private:
    void groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void estimateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishErrorIfReady();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_ground_truth_sub {};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_estimate_sub {};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_error_pub {};

    nav_msgs::msg::Odometry::SharedPtr m_latest_ground_truth {};
    nav_msgs::msg::Odometry::SharedPtr m_latest_estimate {};

    double m_max_stamp_delta {0.2};
    double m_log_period {1.0};
};

#endif
