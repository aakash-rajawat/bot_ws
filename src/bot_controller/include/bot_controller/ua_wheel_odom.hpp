#ifndef UA_WHEEL_ODOM_HPP
#define UA_WHEEL_ODOM_HPP

#include <string>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <bot_interfaces/msg/relative_pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class UAWheelOdom : public rclcpp::Node
{
public:
    explicit UAWheelOdom(const std::string& name);

private:
    using Matrix3d = Eigen::Matrix3d;

    void jointCallback(const sensor_msgs::msg::JointState& msg);
    void computeWheelOdometry(double dp_left, double dp_right, double dt_sec);
    void computeSigmaStaticPrecomputes();
    void updateRelativePoseCovariance(
        double phi_left_previous,
        double phi_left_current,
        double phi_right_previous,
        double phi_right_current
    );
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

    double m_q_static_0 {};
    double m_q_static_1 {};
    double m_q_static_2 {};
    double m_q_static_3 {};
    double m_q_static_4 {};
    double m_q_static_5 {};
    double m_q_static_6 {};
    double m_q_static_7 {};
    double m_q_static_8 {};
    double m_q_static_9 {};
    double m_q_static_10 {};
    double m_q_static_11 {};
    double m_q_static_12 {};
    double m_q_static_13 {};
    double m_q_static_14 {};

    Matrix3d m_increment_pose_covariance {Matrix3d::Zero()};
};


#endif
