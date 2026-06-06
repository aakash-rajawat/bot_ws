#include "bot_evaluation/odom_error.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <cmath>
#include <functional>

namespace
{
double normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

OdomError::OdomError(const std::string& name)
    : Node(name)
{
    declare_parameter<std::string>("ground_truth_topic", "/model/bot/odometry");
    declare_parameter<std::string>("estimate_topic", "/bot_controller/odom_gtsam_fused");
    declare_parameter<double>("max_stamp_delta", 0.2);
    declare_parameter<double>("log_period", 1.0);

    m_max_stamp_delta = get_parameter("max_stamp_delta").as_double();
    m_log_period = get_parameter("log_period").as_double();

    const auto ground_truth_topic = get_parameter("ground_truth_topic").as_string();
    const auto estimate_topic = get_parameter("estimate_topic").as_string();

    constexpr int qos_depth {20};
    m_ground_truth_sub = create_subscription<nav_msgs::msg::Odometry>(
        ground_truth_topic,
        qos_depth,
        std::bind(&OdomError::groundTruthCallback, this, std::placeholders::_1));

    m_estimate_sub = create_subscription<nav_msgs::msg::Odometry>(
        estimate_topic,
        qos_depth,
        std::bind(&OdomError::estimateCallback, this, std::placeholders::_1));

    m_error_pub = create_publisher<nav_msgs::msg::Odometry>(
        "/bot_evaluation/odom_error",
        qos_depth);
}

void OdomError::groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    m_latest_ground_truth = msg;
}

void OdomError::estimateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    m_latest_estimate = msg;
    publishErrorIfReady();
}

void OdomError::publishErrorIfReady()
{
    if(!m_latest_ground_truth || !m_latest_estimate)
    {
        return;
    }

    const rclcpp::Time gt_time(m_latest_ground_truth->header.stamp);
    const rclcpp::Time est_time(m_latest_estimate->header.stamp);
    const double stamp_delta = std::abs((est_time - gt_time).seconds());

    if(m_max_stamp_delta > 0.0 && stamp_delta > m_max_stamp_delta)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Skipping error estimate because stamp delta is %.3f s.",
            stamp_delta);
        return;
    }

    const auto& gt_pose = m_latest_ground_truth->pose.pose;
    const auto& est_pose = m_latest_estimate->pose.pose;

    const double dx = est_pose.position.x - gt_pose.position.x;
    const double dy = est_pose.position.y - gt_pose.position.y;
    const double gt_yaw = tf2::getYaw(gt_pose.orientation);
    const double est_yaw = tf2::getYaw(est_pose.orientation);
    const double dyaw = normalizeAngle(est_yaw - gt_yaw);
    const double position_error = std::hypot(dx, dy);

    nav_msgs::msg::Odometry error_msg;
    error_msg.header.stamp = m_latest_estimate->header.stamp;
    error_msg.header.frame_id = m_latest_estimate->header.frame_id;
    error_msg.child_frame_id = "base_footprint_odom_error";
    error_msg.pose.pose.position.x = dx;
    error_msg.pose.pose.position.y = dy;
    error_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, dyaw);
    error_msg.pose.pose.orientation.x = q.x();
    error_msg.pose.pose.orientation.y = q.y();
    error_msg.pose.pose.orientation.z = q.z();
    error_msg.pose.pose.orientation.w = q.w();

    error_msg.twist.twist.linear.x = position_error;
    error_msg.twist.twist.angular.z = dyaw;

    m_error_pub->publish(error_msg);

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        static_cast<int>(m_log_period * 1000.0),
        "Odom error: dx=%.4f dy=%.4f pos=%.4f dyaw=%.4f rad stamp_delta=%.3f s",
        dx,
        dy,
        position_error,
        dyaw,
        stamp_delta);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomError>("odom_error"));
    rclcpp::shutdown();
    return 0;
}
