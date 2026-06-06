#ifndef SAFETY_STOP_HPP
#define SAFETY_STOP_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <twist_mux_msgs/action/joy_turbo.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>


enum State
{
    FREE = 0,
    WARNING = 1,
    DANGER = 2
};


class SafetyStop : public rclcpp::Node
{
public:
    SafetyStop(const std::string& name);

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_sub {};
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_safety_stop_pub {};

    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr m_decrease_speed_client {};
    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr m_increase_speed_client {};

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_zones_pub {};

    double m_dist_danger {};
    double m_dist_warn {};
    std::string m_scan_topic {};
    std::string m_safety_stop_topic {};
    State m_state {};
    State m_prev_state {};
    bool m_is_first_msg {};
    visualization_msgs::msg::MarkerArray m_zones {};
};

#endif
