#include "bot_controller/ua_wheel_odom.hpp"

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

UAWheelOdom::UAWheelOdom(const std::string& name)
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
    computeSigmaStaticPrecomputes();
    resetCovariances();

    constexpr int subQOS {10};
    m_joint_sub = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        subQOS,
        std::bind(&UAWheelOdom::jointCallback, this, std::placeholders::_1));

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

void UAWheelOdom::resetCovariances()
{
    m_increment_pose_covariance.setZero();
}

void UAWheelOdom::computeWheelOdometry(double dp_left, double dp_right, double dt_sec)
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

void UAWheelOdom::computeSigmaStaticPrecomputes()
{
    const double y_c = m_com_y_offset;
    const double b = m_wheel_separation;
    const double r_L = m_left_wheel_radius;
    const double r_R = m_right_wheel_radius;
    const double sigma_yc = m_com_y_offset_stddev;
    const double sigma_b = m_wheel_separation_stddev;
    const double sigma_rL = m_left_wheel_radius_stddev;
    const double sigma_rR = m_right_wheel_radius_stddev;
    const double sigma_phi_L = m_left_encoder_position_stddev;
    const double sigma_phi_R = m_right_encoder_position_stddev;

    using std::pow;
#include "generated/wheel_odom_sigma_static_precomputes.inl"

    m_q_static_0 = q_static_0;
    m_q_static_1 = q_static_1;
    m_q_static_2 = q_static_2;
    m_q_static_3 = q_static_3;
    m_q_static_4 = q_static_4;
    m_q_static_5 = q_static_5;
    m_q_static_6 = q_static_6;
    m_q_static_7 = q_static_7;
    m_q_static_8 = q_static_8;
    m_q_static_9 = q_static_9;
    m_q_static_10 = q_static_10;
    m_q_static_11 = q_static_11;
    m_q_static_12 = q_static_12;
    m_q_static_13 = q_static_13;
    m_q_static_14 = q_static_14;
}

void UAWheelOdom::updateRelativePoseCovariance(
    double phi_left_previous,
    double phi_left_current,
    double phi_right_previous,
    double phi_right_current)
{
    const double phi_L_previous = phi_left_previous;
    const double phi_L_current = phi_left_current;
    const double phi_R_previous = phi_right_previous;
    const double phi_R_current = phi_right_current;
    const double y_c = m_com_y_offset;
    const double b = m_wheel_separation;
    const double r_L = m_left_wheel_radius;
    const double r_R = m_right_wheel_radius;
    const double q_static_0 = m_q_static_0;
    const double q_static_1 = m_q_static_1;
    const double q_static_2 = m_q_static_2;
    const double q_static_3 = m_q_static_3;
    const double q_static_4 = m_q_static_4;
    const double q_static_5 = m_q_static_5;
    const double q_static_6 = m_q_static_6;
    const double q_static_7 = m_q_static_7;
    const double q_static_8 = m_q_static_8;
    const double q_static_9 = m_q_static_9;
    const double q_static_10 = m_q_static_10;
    const double q_static_11 = m_q_static_11;
    const double q_static_12 = m_q_static_12;
    const double q_static_13 = m_q_static_13;
    const double q_static_14 = m_q_static_14;

    using std::cos;
    using std::pow;
    using std::sin;
#include "generated/wheel_odom_sigma_dynamic_precomputes.inl"

    m_increment_pose_covariance <<
        Sigma_xx, Sigma_xy, Sigma_xyaw,
        Sigma_xy, Sigma_yy, Sigma_yyaw,
        Sigma_xyaw, Sigma_yyaw, Sigma_yawyaw;
    m_increment_pose_covariance +=
        m_increment_pose_covariance_regularization * Matrix3d::Identity();
}

void UAWheelOdom::fillIncrementalPoseMessage(
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

    // Relative wheel increment T_from_to between two timestamped base poses.
    m_incremental_pose_msg.header.stamp = to_stamp;
    m_incremental_pose_msg.from_stamp = from_stamp;
    m_incremental_pose_msg.to_stamp = to_stamp;
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

void UAWheelOdom::jointCallback(const sensor_msgs::msg::JointState& msg)
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

    const double phi_left_previous = m_left_wheel_previous_position;
    const double phi_right_previous = m_right_wheel_previous_position;
    const double phi_left_current = msg.position[left_index];
    const double phi_right_current = msg.position[right_index];
    const double dp_left = phi_left_current - phi_left_previous;
    const double dp_right = phi_right_current - phi_right_previous;

    const rclcpp::Duration dt = msg_time - m_prev_time;
    const double dt_sec = dt.seconds();

    if (dt_sec <= 0.0) {
        m_prev_time = msg_time;
        return;
    }

    const rclcpp::Time previous_stamp = m_prev_time;

    m_left_wheel_previous_position = phi_left_current;
    m_right_wheel_previous_position = phi_right_current;
    m_prev_time = msg_time;

    computeWheelOdometry(dp_left, dp_right, dt_sec);
    updateRelativePoseCovariance(
        phi_left_previous,
        phi_left_current,
        phi_right_previous,
        phi_right_current
    );

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
    rclcpp::spin(std::make_shared<UAWheelOdom>("wheel_odometry_parametric"));
    rclcpp::shutdown();

    return 0;
}
