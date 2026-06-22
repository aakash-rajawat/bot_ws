#ifndef WHEEL_ODOMETRY_PARAMETRIC_HPP
#define WHEEL_ODOMETRY_PARAMETRIC_HPP

#include <string>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <bot_interfaces/msg/relative_pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class WheelOdometryParametric : public rclcpp::Node
{
public:
    explicit WheelOdometryParametric(const std::string& name);

private:
    using Matrix2d = Eigen::Matrix2d;
    using Matrix3d = Eigen::Matrix3d;
    using Matrix2x6d = Eigen::Matrix<double, 2, 6>;
    using Matrix3x2d = Eigen::Matrix<double, 3, 2>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

    void jointCallback(const sensor_msgs::msg::JointState& msg);
    void computeWheelOdometry(double dp_left, double dp_right, double dt_sec);
    void updateTwistCovariance(double wheel_rate_left, double wheel_rate_right, double dt_sec);
    void updateIncrementPoseCovariance(double dt_sec);
    void fillIncrementalPoseMessage(
        const rclcpp::Time& from_stamp,
        const rclcpp::Time& to_stamp,
        double dt_sec
    );
    void resetCovariances();

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_sub {};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_pub {};
    rclcpp::Publisher<bot_interfaces::msg::RelativePoseWithCovarianceStamped>::SharedPtr m_incremental_pose_pub {};
    bot_interfaces::msg::RelativePoseWithCovarianceStamped m_incremental_pose_msg {};
    std::string m_topic_incremental_pose {};

    double m_left_wheel_radius {};
    double m_right_wheel_radius {};
    double m_wheel_separation {};
    double m_com_y_offset {};

    double m_left_wheel_radius_stddev {};
    double m_right_wheel_radius_stddev {};
    double m_wheel_separation_stddev {};
    double m_com_y_offset_stddev {};
    double m_left_encoder_position_stddev {};
    double m_right_encoder_position_stddev {};
    double m_increment_pose_covariance_regularization {};

    double m_unused_pose_variance {};

    double m_left_wheel_previous_position {};
    double m_right_wheel_previous_position {};
    rclcpp::Time m_prev_time {};
    bool m_is_first_joint_state {true};

    double m_pos_x {};
    double m_pos_y {};
    double m_psi {};
    double m_linear_vel {};
    double m_angular_vel {};

    Matrix2d m_twist_covariance {Matrix2d::Zero()};
    Matrix3d m_increment_pose_covariance {Matrix3d::Zero()};
};


#endif
