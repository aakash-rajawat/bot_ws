#ifndef TWIST_RELAY_HPP
#define TWIST_RELAY_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/create_timer.hpp>


class TwistRelay : public rclcpp::Node
{
public:
    TwistRelay(const std::string& name);

private:
    void controllerTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void joyTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void controllerWatchdogCallback();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_controller_sub {};
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_controller_pub {};

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_joy_sub {};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_joy_pub {};

    rclcpp::TimerBase::SharedPtr m_controller_watchdog_timer {};
    rclcpp::Time m_last_controller_cmd_time {};
    bool m_has_controller_cmd {false};
    bool m_controller_zero_sent {false};

    double m_controller_cmd_timeout {0.2};
};


#endif
