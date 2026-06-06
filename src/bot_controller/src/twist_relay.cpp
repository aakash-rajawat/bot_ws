#include "bot_controller/twist_relay.hpp"

TwistRelay::TwistRelay(const std::string& name)
    : Node(name)
{
    declare_parameter<double>("controller_cmd_timeout", 0.2);
    m_controller_cmd_timeout = get_parameter("controller_cmd_timeout").as_double();

    constexpr int subQOS {10};
    m_controller_sub = create_subscription<geometry_msgs::msg::Twist>(
        "/bot_controller/cmd_vel_unstamped",
        subQOS,
        std::bind(&TwistRelay::controllerTwistCallback, this, std::placeholders::_1)
    );

    constexpr int pubQOS {10};
    m_controller_pub = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/bot_controller/cmd_vel",
        pubQOS
    );

    m_joy_sub = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/input_joy/cmd_vel_stamped",
        subQOS,
        std::bind(&TwistRelay::joyTwistCallback, this, std::placeholders::_1)
    );

    m_joy_pub = create_publisher<geometry_msgs::msg::Twist>(
        "/input_joy/cmd_vel",
        pubQOS
    );

    m_controller_watchdog_timer = rclcpp::create_timer(
        this,
        get_clock(),
        std::chrono::milliseconds(50),
        std::bind(&TwistRelay::controllerWatchdogCallback, this)
    );
}

void TwistRelay::controllerTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    geometry_msgs::msg::TwistStamped twist_stamped {};
    twist_stamped.header.stamp = get_clock()->now();
    twist_stamped.twist = *msg;

    m_last_controller_cmd_time = get_clock()->now();
    m_has_controller_cmd = true;
    m_controller_zero_sent = false;

    m_controller_pub->publish(twist_stamped);
}

void TwistRelay::joyTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    geometry_msgs::msg::Twist twist {};
    twist = msg->twist;
    m_joy_pub->publish(twist);
}

void TwistRelay::controllerWatchdogCallback()
{
    if (!m_has_controller_cmd || m_controller_zero_sent) {
        return;
    }

    const auto now = get_clock()->now();
    const double age = (now - m_last_controller_cmd_time).seconds();

    if (age < m_controller_cmd_timeout) {
        return;
    }

    geometry_msgs::msg::TwistStamped zero_twist {};
    zero_twist.header.stamp = now;
    zero_twist.twist.linear.x = 0.0;
    zero_twist.twist.linear.y = 0.0;
    zero_twist.twist.linear.z = 0.0;
    zero_twist.twist.angular.x = 0.0;
    zero_twist.twist.angular.y = 0.0;
    zero_twist.twist.angular.z = 0.0;

    m_controller_pub->publish(zero_twist);
    m_controller_zero_sent = true;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistRelay>("twist_relay"));
    rclcpp::shutdown();

    return 0;
}
