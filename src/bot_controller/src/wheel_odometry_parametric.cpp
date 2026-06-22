#include "bot_controller/wheel_odometry_parametric.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

namespace
{
    bool isValidCovariance3(const Eigen::Matrix3d& Q)
    {
        if(!Q.allFinite())
        {
            return false;
        }

        const Eigen::Matrix3d Q_sym = 0.5 * (Q + Q.transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(Q_sym);
        if(solver.info() != Eigen::Success)
        {
            return false;
        }

        constexpr double kMinEig {-1.0e-12};
        return solver.eigenvalues().minCoeff() >= kMinEig;
    }
}

WheelOdometryParametric::WheelOdometryParametric(const std::string& name)
    : Node(name),
      m_left_wheel_previous_position(0.0),
      m_right_wheel_previous_position(0.0),
      m_pos_x(0.0),
      m_pos_y(0.0),
      m_psi(0.0),
      m_linear_vel(0.0),
      m_angular_vel(0.0)
{
    declare_parameter("left_wheel_radius", 0.033);
    m_left_wheel_radius = get_parameter("left_wheel_radius").as_double();

    declare_parameter("right_wheel_radius", 0.033);
    m_right_wheel_radius = get_parameter("right_wheel_radius").as_double();

    declare_parameter("wheel_separation", 0.17);
    m_wheel_separation = get_parameter("wheel_separation").as_double();

    declare_parameter("com_y_offset", 0.0);
    m_com_y_offset = get_parameter("com_y_offset").as_double();

    declare_parameter("left_wheel_radius_stddev", 0.001);
    m_left_wheel_radius_stddev = get_parameter("left_wheel_radius_stddev").as_double();

    declare_parameter("right_wheel_radius_stddev", 0.001);
    m_right_wheel_radius_stddev = get_parameter("right_wheel_radius_stddev").as_double();

    declare_parameter("wheel_separation_stddev", 0.001);
    m_wheel_separation_stddev = get_parameter("wheel_separation_stddev").as_double();

    declare_parameter("com_y_offset_stddev", 0.0005);
    m_com_y_offset_stddev = get_parameter("com_y_offset_stddev").as_double();

    declare_parameter("left_encoder_position_stddev", 0.0018);
    m_left_encoder_position_stddev = get_parameter("left_encoder_position_stddev").as_double();

    declare_parameter("right_encoder_position_stddev", 0.0018);
    m_right_encoder_position_stddev = get_parameter("right_encoder_position_stddev").as_double();

    declare_parameter("increment_pose_covariance_regularization", 1.0e-9);
    m_increment_pose_covariance_regularization =
        std::max(0.0, get_parameter("increment_pose_covariance_regularization").as_double());

    declare_parameter("unused_pose_variance", 1.0e6);
    m_unused_pose_variance = get_parameter("unused_pose_variance").as_double();

    declare_parameter<std::string>(
        "topic_incremental_pose",
        "/bot_controller/relative_pose_wheel"
    );
    m_topic_incremental_pose = get_parameter("topic_incremental_pose").as_string();

    m_prev_time = get_clock()->now();
    resetCovariances();

    constexpr int subQOS {10};
    m_joint_sub = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        subQOS,
        std::bind(&WheelOdometryParametric::jointCallback, this, std::placeholders::_1));

    constexpr int pubQOS {10};
    m_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/wheel_odometry/pose",
        pubQOS
    );

    m_incremental_pose_pub = create_publisher<bot_interfaces::msg::RelativePoseWithCovarianceStamped>(
        m_topic_incremental_pose,
        pubQOS
    );

    RCLCPP_INFO_STREAM(get_logger(),
                       "wheel_odometry_parametric node initialised with\n"
                           << "left wheel radius: " << m_left_wheel_radius << " m\n"
                           << "right wheel radius: " << m_right_wheel_radius << " m\n"
                           << "wheel separation: " << m_wheel_separation << " m\n"
                           << "center of mass y offset: " << m_com_y_offset << " m\n"
                           << "left radius stddev: " << m_left_wheel_radius_stddev << " m\n"
                           << "right radius stddev: " << m_right_wheel_radius_stddev << " m\n"
                           << "wheel separation stddev: " << m_wheel_separation_stddev << " m\n"
                           << "center of mass y offset stddev: " << m_com_y_offset_stddev << " m\n"
                           << "left encoder position stddev: " << m_left_encoder_position_stddev << " rad\n"
                           << "right encoder position stddev: " << m_right_encoder_position_stddev << " rad\n"
                           << "increment pose covariance regularization: "
                           << m_increment_pose_covariance_regularization);
}

void WheelOdometryParametric::resetCovariances()
{
    m_twist_covariance.setZero();
    m_increment_pose_covariance.setZero();
}

void WheelOdometryParametric::computeWheelOdometry(double dp_left, double dp_right, double dt_sec)
{
    const double left_wheel_angular_velocity = dp_left / dt_sec;
    const double right_wheel_angular_velocity = dp_right / dt_sec;

    const double wheel_difference =
        (m_right_wheel_radius * right_wheel_angular_velocity) -
        (m_left_wheel_radius * left_wheel_angular_velocity);

    m_linear_vel =
        ((m_right_wheel_radius * right_wheel_angular_velocity) +
         (m_left_wheel_radius * left_wheel_angular_velocity)) / 2.0 +
        (m_com_y_offset * wheel_difference) / m_wheel_separation;

    m_angular_vel = wheel_difference / m_wheel_separation;

    const double midpoint_heading = m_psi + (dt_sec * m_angular_vel / 2.0);

    m_pos_x += dt_sec * m_linear_vel * std::cos(midpoint_heading);
    m_pos_y += dt_sec * m_linear_vel * std::sin(midpoint_heading);
    m_psi += dt_sec * m_angular_vel;
}

void WheelOdometryParametric::updateTwistCovariance(
    double wheel_rate_left,
    double wheel_rate_right,
    double dt_sec)
{
    const double sigma_w_left = std::sqrt(2.0) * m_left_encoder_position_stddev / dt_sec;
    const double sigma_w_right = std::sqrt(2.0) * m_right_encoder_position_stddev / dt_sec;

    Matrix6d q = Matrix6d::Zero();
    q(0, 0) = m_com_y_offset_stddev * m_com_y_offset_stddev;
    q(1, 1) = m_wheel_separation_stddev * m_wheel_separation_stddev;
    q(2, 2) = m_left_wheel_radius_stddev * m_left_wheel_radius_stddev;
    q(3, 3) = m_right_wheel_radius_stddev * m_right_wheel_radius_stddev;
    q(4, 4) = sigma_w_left * sigma_w_left;
    q(5, 5) = sigma_w_right * sigma_w_right;

    Matrix2x6d j_tw = Matrix2x6d::Zero();
    j_tw(0, 0) = ((m_right_wheel_radius * wheel_rate_right) -
                  (m_left_wheel_radius * wheel_rate_left)) /
                 m_wheel_separation;
    j_tw(0, 1) = -m_com_y_offset *
                 ((m_right_wheel_radius * wheel_rate_right) -
                  (m_left_wheel_radius * wheel_rate_left)) /
                 (m_wheel_separation * m_wheel_separation);
    j_tw(0, 2) = wheel_rate_left * (0.5 - (m_com_y_offset / m_wheel_separation));
    j_tw(0, 3) = wheel_rate_right * (0.5 + (m_com_y_offset / m_wheel_separation));
    j_tw(0, 4) = m_left_wheel_radius * (0.5 - (m_com_y_offset / m_wheel_separation));
    j_tw(0, 5) = m_right_wheel_radius * (0.5 + (m_com_y_offset / m_wheel_separation));

    j_tw(1, 0) = 0.0;
    j_tw(1, 1) = -((m_right_wheel_radius * wheel_rate_right) -
                   (m_left_wheel_radius * wheel_rate_left)) /
                 (m_wheel_separation * m_wheel_separation);
    j_tw(1, 2) = -wheel_rate_left / m_wheel_separation;
    j_tw(1, 3) = wheel_rate_right / m_wheel_separation;
    j_tw(1, 4) = -m_left_wheel_radius / m_wheel_separation;
    j_tw(1, 5) = m_right_wheel_radius / m_wheel_separation;

    m_twist_covariance = j_tw * q * j_tw.transpose();
}

void WheelOdometryParametric::updateIncrementPoseCovariance(double dt_sec)
{
    const double increment_alpha = 0.5 * dt_sec * m_angular_vel;
    const double cos_increment_alpha = std::cos(increment_alpha);
    const double sin_increment_alpha = std::sin(increment_alpha);

    Matrix3x2d l_increment = Matrix3x2d::Zero();
    l_increment(0, 0) = dt_sec * cos_increment_alpha;
    l_increment(0, 1) = -0.5 * dt_sec * dt_sec * m_linear_vel * sin_increment_alpha;
    l_increment(1, 0) = dt_sec * sin_increment_alpha;
    l_increment(1, 1) = 0.5 * dt_sec * dt_sec * m_linear_vel * cos_increment_alpha;
    l_increment(2, 1) = dt_sec;

    m_increment_pose_covariance = l_increment * m_twist_covariance * l_increment.transpose();
    m_increment_pose_covariance +=
        m_increment_pose_covariance_regularization * Matrix3d::Identity();
}

void WheelOdometryParametric::fillIncrementalPoseMessage(
    const rclcpp::Time& from_stamp,
    const rclcpp::Time& to_stamp,
    double dt_sec
)
{
    const double delta_yaw = dt_sec * m_angular_vel;
    const double delta_s = dt_sec * m_linear_vel;
    const double midpoint_yaw = 0.5 * delta_yaw;

    tf2::Quaternion q {};
    q.setRPY(0.0, 0.0, delta_yaw);

    // Relative wheel increment T_from_to. Both endpoint frames are the
    // robot body frame at different times; the pose maps coordinates
    // from to_frame into from_frame.
    m_incremental_pose_msg.header.stamp = to_stamp;
    m_incremental_pose_msg.header.frame_id = "base_footprint_noisy_temp";
    m_incremental_pose_msg.from_stamp = from_stamp;
    m_incremental_pose_msg.to_stamp = to_stamp;
    m_incremental_pose_msg.from_frame_id = "base_footprint_noisy_temp";
    m_incremental_pose_msg.to_frame_id = "base_footprint_noisy_temp";
    m_incremental_pose_msg.pose.pose.position.x = delta_s * std::cos(midpoint_yaw);
    m_incremental_pose_msg.pose.pose.position.y = delta_s * std::sin(midpoint_yaw);
    m_incremental_pose_msg.pose.pose.position.z = 0.0;
    m_incremental_pose_msg.pose.pose.orientation.x = q.x();
    m_incremental_pose_msg.pose.pose.orientation.y = q.y();
    m_incremental_pose_msg.pose.pose.orientation.z = q.z();
    m_incremental_pose_msg.pose.pose.orientation.w = q.w();

    const Matrix3d Q_inc = 0.5 * (
        m_increment_pose_covariance + m_increment_pose_covariance.transpose()
    );

    m_incremental_pose_msg.pose.covariance.fill(0.0);
    m_incremental_pose_msg.pose.covariance[0] = Q_inc(0, 0);
    m_incremental_pose_msg.pose.covariance[1] = Q_inc(0, 1);
    m_incremental_pose_msg.pose.covariance[5] = Q_inc(0, 2);
    m_incremental_pose_msg.pose.covariance[6] = Q_inc(1, 0);
    m_incremental_pose_msg.pose.covariance[7] = Q_inc(1, 1);
    m_incremental_pose_msg.pose.covariance[11] = Q_inc(1, 2);
    m_incremental_pose_msg.pose.covariance[30] = Q_inc(2, 0);
    m_incremental_pose_msg.pose.covariance[31] = Q_inc(2, 1);
    m_incremental_pose_msg.pose.covariance[35] = Q_inc(2, 2);
    m_incremental_pose_msg.pose.covariance[14] = m_unused_pose_variance;
    m_incremental_pose_msg.pose.covariance[21] = m_unused_pose_variance;
    m_incremental_pose_msg.pose.covariance[28] = m_unused_pose_variance;
}

void WheelOdometryParametric::jointCallback(const sensor_msgs::msg::JointState& msg)
{
    auto left_it = std::find(msg.name.begin(), msg.name.end(), "base_lw_joint");
    auto right_it = std::find(msg.name.begin(), msg.name.end(), "base_rw_joint");

    if (left_it == msg.name.end() || right_it == msg.name.end()) {
        RCLCPP_WARN(get_logger(), "Wheel joint names not found in /joint_states");
        return;
    }

    const auto left_index = std::distance(msg.name.begin(), left_it);
    const auto right_index = std::distance(msg.name.begin(), right_it);

    if (left_index >= static_cast<long>(msg.position.size()) ||
        right_index >= static_cast<long>(msg.position.size())) {
        RCLCPP_WARN(get_logger(), "Wheel joint positions missing in /joint_states");
        return;
    }

    const rclcpp::Time msg_time = msg.header.stamp;

    if (m_is_first_joint_state) {
        m_left_wheel_previous_position = msg.position[left_index];
        m_right_wheel_previous_position = msg.position[right_index];
        m_prev_time = msg_time;
        m_is_first_joint_state = false;
        return;
    }

    const double dp_left = msg.position[left_index] - m_left_wheel_previous_position;
    const double dp_right = msg.position[right_index] - m_right_wheel_previous_position;

    const rclcpp::Duration dt = msg_time - m_prev_time;
    const double dt_sec = dt.seconds();

    if (dt_sec <= 0.0) {
        m_prev_time = msg_time;
        return;
    }

    const rclcpp::Time previous_stamp = m_prev_time;
    const double wheel_rate_left = dp_left / dt_sec;
    const double wheel_rate_right = dp_right / dt_sec;

    m_left_wheel_previous_position = msg.position[left_index];
    m_right_wheel_previous_position = msg.position[right_index];
    m_prev_time = msg_time;

    computeWheelOdometry(dp_left, dp_right, dt_sec);
    updateTwistCovariance(wheel_rate_left, wheel_rate_right, dt_sec);
    updateIncrementPoseCovariance(dt_sec);

    geometry_msgs::msg::PoseStamped stamped_pose {};
    stamped_pose.header.stamp = msg_time;
    stamped_pose.header.frame_id = "odom";
    stamped_pose.pose.position.x = m_pos_x;
    stamped_pose.pose.position.y = m_pos_y;
    stamped_pose.pose.position.z = 0.0;

    tf2::Quaternion q {};
    q.setRPY(0.0, 0.0, m_psi);
    stamped_pose.pose.orientation.x = q.x();
    stamped_pose.pose.orientation.y = q.y();
    stamped_pose.pose.orientation.z = q.z();
    stamped_pose.pose.orientation.w = q.w();
    m_pose_pub->publish(stamped_pose);

    if(!isValidCovariance3(m_increment_pose_covariance))
    {
        RCLCPP_WARN(
            get_logger(),
            "Skipping wheel incremental pose publish because increment covariance is invalid."
        );
        return;
    }

    fillIncrementalPoseMessage(previous_stamp, msg_time, dt_sec);
    m_incremental_pose_pub->publish(m_incremental_pose_msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometryParametric>("wheel_odometry_parametric"));
    rclcpp::shutdown();

    return 0;
}
