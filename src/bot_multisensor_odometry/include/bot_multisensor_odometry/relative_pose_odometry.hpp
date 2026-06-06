#ifndef RELATIVE_POSE_ODOMETRY_HPP
#define RELATIVE_POSE_ODOMETRY_HPP

#include <rclcpp/rclcpp.hpp>
#include <bot_interfaces/msg/relative_pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <optional>
#include <Eigen/Dense>

namespace bot_multisensor_odometry::relative_pose_odometry
{
    class RelativePoseOdometry : public rclcpp::Node
    {
    public:
        RelativePoseOdometry(const std::string& name);

    private:
        using Cov6 = Eigen::Matrix<double, 6, 6>;

        void poseStampedCallback(
            const bot_interfaces::msg::RelativePoseWithCovarianceStamped& stamped_pose_msg
        );

        rclcpp::Subscription<bot_interfaces::msg::RelativePoseWithCovarianceStamped>::SharedPtr
            m_mle_pose_sub {};
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub {};

        std::string m_mle_pose_topic_name {};
        std::string m_odom_pub_topic_name {};

        nav_msgs::msg::Odometry m_odom_msg {};
        std::optional<rclcpp::Time> m_prev_stamp {};
        Cov6 m_pose_covariance = Cov6::Zero();
        bool m_has_pose_covariance {false};
    };
}

#endif
