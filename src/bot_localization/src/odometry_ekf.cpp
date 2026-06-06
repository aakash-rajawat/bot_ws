#include "bot_localization/odometry_ekf.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

namespace
{
template<typename MatrixType>
void regularizeInnovationCovariance(MatrixType& innovation_covariance)
{
    // Remove tiny asymmetry introduced by floating-point roundoff.
    innovation_covariance =
        0.5 * (innovation_covariance + innovation_covariance.transpose());

    // Add a small diagonal jitter so near-singular innovations remain solvable.
    innovation_covariance +=
        1.0e-9 * MatrixType::Identity(innovation_covariance.rows(), innovation_covariance.cols());
}

template<typename MatrixType>
bool factorizeInnovationCovariance(
    MatrixType& innovation_covariance,
    Eigen::LDLT<MatrixType>& ldlt)
{
    regularizeInnovationCovariance(innovation_covariance);
    ldlt.compute(innovation_covariance);
    return ldlt.info() == Eigen::Success;
}
}  // namespace

OdometryEkf::OdometryEkf(const std::string& name)
    : Node(name)
{
    declare_parameter("use_imu_orientation", true);
    m_use_imu_orientation = get_parameter("use_imu_orientation").as_bool();

    declare_parameter("use_imu_yaw_rate", true);
    m_use_imu_yaw_rate = get_parameter("use_imu_yaw_rate").as_bool();

    declare_parameter("output_frame", std::string("odom"));
    m_output_frame = get_parameter("output_frame").as_string();

    declare_parameter("output_child_frame", std::string("base_footprint_ekf"));
    m_output_child_frame = get_parameter("output_child_frame").as_string();

    declare_parameter("min_variance", 1.0e-9);
    m_min_variance = get_parameter("min_variance").as_double();

    declare_parameter("unused_pose_variance", 1.0e6);
    m_unused_pose_variance = get_parameter("unused_pose_variance").as_double();

    declare_parameter("unused_twist_variance", 1.0e6);
    m_unused_twist_variance = get_parameter("unused_twist_variance").as_double();

    declare_parameter("imu_yaw_variance_override", 0.05);
    m_imu_yaw_variance_override = get_parameter("imu_yaw_variance_override").as_double();

    declare_parameter("imu_yaw_rate_variance_override", 1.0e-5);
    m_imu_yaw_rate_variance_override =
        get_parameter("imu_yaw_rate_variance_override").as_double();

    declare_parameter("process_noise_x", 0.02);
    declare_parameter("process_noise_y", 0.02);
    declare_parameter("process_noise_yaw", 0.01);
    declare_parameter("process_noise_v", 0.05);
    declare_parameter("process_noise_omega", 0.05);

    m_process_noise.setZero();
    m_process_noise(0, 0) = get_parameter("process_noise_x").as_double();
    m_process_noise(1, 1) = get_parameter("process_noise_y").as_double();
    m_process_noise(2, 2) = get_parameter("process_noise_yaw").as_double();
    m_process_noise(3, 3) = get_parameter("process_noise_v").as_double();
    m_process_noise(4, 4) = get_parameter("process_noise_omega").as_double();

    constexpr int qos_depth {10};
    m_odom_sub = create_subscription<nav_msgs::msg::Odometry>(
        "/bot_controller/odom_noisy_temp",
        qos_depth,
        std::bind(&OdometryEkf::odomCallback, this, std::placeholders::_1));

    m_imu_sub = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/out",
        qos_depth,
        std::bind(&OdometryEkf::imuCallback, this, std::placeholders::_1));

    m_odom_pub = create_publisher<nav_msgs::msg::Odometry>(
        "/bot_controller/odom_ekf",
        qos_depth);

    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    m_odom_msg.header.frame_id = m_output_frame;
    m_odom_msg.child_frame_id = m_output_child_frame;
    m_tf_msg.header.frame_id = m_output_frame;
    m_tf_msg.child_frame_id = m_output_child_frame;

    RCLCPP_INFO_STREAM(
        get_logger(),
        "odometry_ekf node initialised with\n"
            << "use imu orientation: " << std::boolalpha << m_use_imu_orientation << "\n"
            << "use imu yaw rate: " << std::boolalpha << m_use_imu_yaw_rate << "\n"
            << "output frame: " << m_output_frame << "\n"
            << "output child frame: " << m_output_child_frame);
}

double OdometryEkf::normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double OdometryEkf::quaternionToYaw(const geometry_msgs::msg::Quaternion& q)
{
    const double siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y));
    const double cosy_cosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)));

    return std::atan2(siny_cosp, cosy_cosp);
}

void OdometryEkf::initializeFromOdometry(const nav_msgs::msg::Odometry& msg)
{
    m_state.setZero();
    m_state(0) = msg.pose.pose.position.x;
    m_state(1) = msg.pose.pose.position.y;
    m_state(2) = quaternionToYaw(msg.pose.pose.orientation);
    m_state(3) = msg.twist.twist.linear.x;
    m_state(4) = msg.twist.twist.angular.z;
    m_state(2) = normalizeAngle(m_state(2));

    m_covariance = buildOdometryMeasurementCovariance(msg);
    m_last_predict_time = msg.header.stamp;
    m_is_initialized = true;

    publishEstimate(msg.header.stamp);
}

void OdometryEkf::predictTo(const rclcpp::Time& stamp)
{
    if (!m_is_initialized) {
        return;
    }

    const double dt = (stamp - m_last_predict_time).seconds();
    if (dt <= 0.0) {
        return;
    }

    const double x = m_state(0);
    const double y = m_state(1);
    const double yaw = m_state(2);
    const double v = m_state(3);
    const double omega = m_state(4);

    const double alpha = yaw + (0.5 * dt * omega);
    const double cos_alpha = std::cos(alpha);
    const double sin_alpha = std::sin(alpha);

    Vector5d predicted = m_state;
    predicted(0) = x + (dt * v * cos_alpha);
    predicted(1) = y + (dt * v * sin_alpha);
    predicted(2) = normalizeAngle(yaw + (dt * omega));
    predicted(3) = v;
    predicted(4) = omega;

    Matrix5d f = Matrix5d::Identity();
    f(0, 2) = -dt * v * sin_alpha;
    f(0, 3) = dt * cos_alpha;
    f(0, 4) = -0.5 * dt * dt * v * sin_alpha;
    f(1, 2) = dt * v * cos_alpha;
    f(1, 3) = dt * sin_alpha;
    f(1, 4) = 0.5 * dt * dt * v * cos_alpha;
    f(2, 4) = dt;

    m_state = predicted;
    m_covariance = (f * m_covariance * f.transpose()) + (m_process_noise * dt);
    m_covariance = 0.5 * (m_covariance + m_covariance.transpose());

    m_last_predict_time = stamp;
}

OdometryEkf::Matrix5d OdometryEkf::buildOdometryMeasurementCovariance(
    const nav_msgs::msg::Odometry& msg) const
{
    Matrix5d r = Matrix5d::Zero();

    const auto& pose_cov = msg.pose.covariance;
    const auto& twist_cov = msg.twist.covariance;

    r(0, 0) = std::max(pose_cov[0], m_min_variance);
    r(0, 1) = pose_cov[1];
    r(0, 2) = pose_cov[5];

    r(1, 0) = pose_cov[6];
    r(1, 1) = std::max(pose_cov[7], m_min_variance);
    r(1, 2) = pose_cov[11];

    r(2, 0) = pose_cov[30];
    r(2, 1) = pose_cov[31];
    r(2, 2) = std::max(pose_cov[35], m_min_variance);

    r(3, 3) = std::max(twist_cov[0], m_min_variance);
    r(3, 4) = twist_cov[5];
    r(4, 3) = twist_cov[30];
    r(4, 4) = std::max(twist_cov[35], m_min_variance);

    r = 0.5 * (r + r.transpose());

    return r;
}

double OdometryEkf::getImuYawVariance(const sensor_msgs::msg::Imu& msg) const
{
    const double variance = msg.orientation_covariance[8];
    if (variance > 0.0) {
        return variance;
    }

    return m_imu_yaw_variance_override;
}

double OdometryEkf::getImuYawRateVariance(const sensor_msgs::msg::Imu& msg) const
{
    const double variance = msg.angular_velocity_covariance[8];
    if (variance > 0.0) {
        return variance;
    }

    return m_imu_yaw_rate_variance_override;
}

void OdometryEkf::updateFromOdometry(const nav_msgs::msg::Odometry& msg)
{
    Matrix5d h = Matrix5d::Identity();
    Matrix5d r = buildOdometryMeasurementCovariance(msg);

    Vector5d z = Vector5d::Zero();
    z(0) = msg.pose.pose.position.x;
    z(1) = msg.pose.pose.position.y;
    z(2) = quaternionToYaw(msg.pose.pose.orientation);
    z(3) = msg.twist.twist.linear.x;
    z(4) = msg.twist.twist.angular.z;

    Vector5d innovation = z - (h * m_state);
    innovation(2) = normalizeAngle(innovation(2));

    Matrix5d s = (h * m_covariance * h.transpose()) + r;
    Eigen::LDLT<Matrix5d> ldlt {};

    // Skip the measurement update if the innovation covariance is numerically invalid.
    if (!factorizeInnovationCovariance(s, ldlt)) {
        RCLCPP_WARN(
            get_logger(),
            "Skipping odometry update because innovation covariance factorization failed");
        return;
    }

    Matrix5d k = m_covariance * h.transpose() * ldlt.solve(Matrix5d::Identity());

    m_state = m_state + (k * innovation);
    m_state(2) = normalizeAngle(m_state(2));

    const Matrix5d identity = Matrix5d::Identity();
    m_covariance =
        ((identity - (k * h)) * m_covariance * (identity - (k * h)).transpose()) +
        (k * r * k.transpose());
    m_covariance = 0.5 * (m_covariance + m_covariance.transpose());
}

void OdometryEkf::updateFromImu(const sensor_msgs::msg::Imu& msg)
{
    if (m_use_imu_orientation && m_use_imu_yaw_rate) {
        Matrix2x5d h = Matrix2x5d::Zero();
        h(0, 2) = 1.0;
        h(1, 4) = 1.0;

        Vector2d z = Vector2d::Zero();
        z(0) = quaternionToYaw(msg.orientation);
        z(1) = msg.angular_velocity.z;

        Matrix2d r = Matrix2d::Zero();
        r(0, 0) = std::max(getImuYawVariance(msg), m_min_variance);
        r(1, 1) = std::max(getImuYawRateVariance(msg), m_min_variance);

        Vector2d innovation = z - (h * m_state);
        innovation(0) = normalizeAngle(innovation(0));

        Matrix2d s = (h * m_covariance * h.transpose()) + r;
        Eigen::LDLT<Matrix2d> ldlt {};

        // Reject this IMU update if the innovation covariance is not safely factorizable.
        if (!factorizeInnovationCovariance(s, ldlt)) {
            RCLCPP_WARN(
                get_logger(),
                "Skipping IMU yaw/yaw-rate update because innovation covariance factorization failed");
            return;
        }

        Eigen::Matrix<double, 5, 2> k =
            m_covariance * h.transpose() * ldlt.solve(Matrix2d::Identity());

        m_state = m_state + (k * innovation);
        m_state(2) = normalizeAngle(m_state(2));

        const Matrix5d identity = Matrix5d::Identity();
        m_covariance =
            ((identity - (k * h)) * m_covariance * (identity - (k * h)).transpose()) +
            (k * r * k.transpose());
        m_covariance = 0.5 * (m_covariance + m_covariance.transpose());
        return;
    }

    if (m_use_imu_orientation) {
        Matrix1x5d h = Matrix1x5d::Zero();
        h(0, 2) = 1.0;

        Vector1d z = Vector1d::Zero();
        z(0) = quaternionToYaw(msg.orientation);

        Matrix1d r = Matrix1d::Zero();
        r(0, 0) = std::max(getImuYawVariance(msg), m_min_variance);

        Vector1d innovation = z - (h * m_state);
        innovation(0) = normalizeAngle(innovation(0));

        Matrix1d s = (h * m_covariance * h.transpose()) + r;
        Eigen::LDLT<Matrix1d> ldlt {};

        // Reject this IMU update if the innovation covariance is not safely factorizable.
        if (!factorizeInnovationCovariance(s, ldlt)) {
            RCLCPP_WARN(
                get_logger(),
                "Skipping IMU yaw update because innovation covariance factorization failed");
            return;
        }

        Eigen::Matrix<double, 5, 1> k =
            m_covariance * h.transpose() * ldlt.solve(Matrix1d::Identity());

        m_state = m_state + (k * innovation);
        m_state(2) = normalizeAngle(m_state(2));

        const Matrix5d identity = Matrix5d::Identity();
        m_covariance =
            ((identity - (k * h)) * m_covariance * (identity - (k * h)).transpose()) +
            (k * r * k.transpose());
        m_covariance = 0.5 * (m_covariance + m_covariance.transpose());
        return;
    }

    if (m_use_imu_yaw_rate) {
        Matrix1x5d h = Matrix1x5d::Zero();
        h(0, 4) = 1.0;

        Vector1d z = Vector1d::Zero();
        z(0) = msg.angular_velocity.z;

        Matrix1d r = Matrix1d::Zero();
        r(0, 0) = std::max(getImuYawRateVariance(msg), m_min_variance);

        Vector1d innovation = z - (h * m_state);

        Matrix1d s = (h * m_covariance * h.transpose()) + r;
        Eigen::LDLT<Matrix1d> ldlt {};

        // Reject this IMU update if the innovation covariance is not safely factorizable.
        if (!factorizeInnovationCovariance(s, ldlt)) {
            RCLCPP_WARN(
                get_logger(),
                "Skipping IMU yaw-rate update because innovation covariance factorization failed");
            return;
        }

        Eigen::Matrix<double, 5, 1> k =
            m_covariance * h.transpose() * ldlt.solve(Matrix1d::Identity());

        m_state = m_state + (k * innovation);
        m_state(2) = normalizeAngle(m_state(2));

        const Matrix5d identity = Matrix5d::Identity();
        m_covariance =
            ((identity - (k * h)) * m_covariance * (identity - (k * h)).transpose()) +
            (k * r * k.transpose());
        m_covariance = 0.5 * (m_covariance + m_covariance.transpose());
    }
}

void OdometryEkf::publishEstimate(const rclcpp::Time& stamp)
{
    tf2::Quaternion q {};
    q.setRPY(0.0, 0.0, m_state(2));

    m_odom_msg.header.stamp = stamp;
    m_odom_msg.header.frame_id = m_output_frame;
    m_odom_msg.child_frame_id = m_output_child_frame;

    m_odom_msg.pose.pose.position.x = m_state(0);
    m_odom_msg.pose.pose.position.y = m_state(1);
    m_odom_msg.pose.pose.position.z = 0.0;
    m_odom_msg.pose.pose.orientation.x = q.x();
    m_odom_msg.pose.pose.orientation.y = q.y();
    m_odom_msg.pose.pose.orientation.z = q.z();
    m_odom_msg.pose.pose.orientation.w = q.w();

    m_odom_msg.twist.twist.linear.x = m_state(3);
    m_odom_msg.twist.twist.linear.y = 0.0;
    m_odom_msg.twist.twist.linear.z = 0.0;
    m_odom_msg.twist.twist.angular.x = 0.0;
    m_odom_msg.twist.twist.angular.y = 0.0;
    m_odom_msg.twist.twist.angular.z = m_state(4);

    m_odom_msg.pose.covariance.fill(0.0);
    m_odom_msg.twist.covariance.fill(0.0);

    m_odom_msg.pose.covariance[0] = m_covariance(0, 0);
    m_odom_msg.pose.covariance[1] = m_covariance(0, 1);
    m_odom_msg.pose.covariance[5] = m_covariance(0, 2);
    m_odom_msg.pose.covariance[6] = m_covariance(1, 0);
    m_odom_msg.pose.covariance[7] = m_covariance(1, 1);
    m_odom_msg.pose.covariance[11] = m_covariance(1, 2);
    m_odom_msg.pose.covariance[30] = m_covariance(2, 0);
    m_odom_msg.pose.covariance[31] = m_covariance(2, 1);
    m_odom_msg.pose.covariance[35] = m_covariance(2, 2);
    m_odom_msg.pose.covariance[14] = m_unused_pose_variance;
    m_odom_msg.pose.covariance[21] = m_unused_pose_variance;
    m_odom_msg.pose.covariance[28] = m_unused_pose_variance;

    m_odom_msg.twist.covariance[0] = m_covariance(3, 3);
    m_odom_msg.twist.covariance[5] = m_covariance(3, 4);
    m_odom_msg.twist.covariance[30] = m_covariance(4, 3);
    m_odom_msg.twist.covariance[35] = m_covariance(4, 4);
    m_odom_msg.twist.covariance[7] = m_unused_twist_variance;
    m_odom_msg.twist.covariance[14] = m_unused_twist_variance;
    m_odom_msg.twist.covariance[21] = m_unused_twist_variance;
    m_odom_msg.twist.covariance[28] = m_unused_twist_variance;

    m_tf_msg.header.stamp = stamp;
    m_tf_msg.header.frame_id = m_output_frame;
    m_tf_msg.child_frame_id = m_output_child_frame;
    m_tf_msg.transform.translation.x = m_state(0);
    m_tf_msg.transform.translation.y = m_state(1);
    m_tf_msg.transform.translation.z = 0.0;
    m_tf_msg.transform.rotation.x = q.x();
    m_tf_msg.transform.rotation.y = q.y();
    m_tf_msg.transform.rotation.z = q.z();
    m_tf_msg.transform.rotation.w = q.w();

    m_odom_pub->publish(m_odom_msg);
    m_tf_broadcaster->sendTransform(m_tf_msg);
}

void OdometryEkf::odomCallback(const nav_msgs::msg::Odometry& msg)
{
    if (!m_is_initialized) {
        initializeFromOdometry(msg);
        return;
    }

    predictTo(msg.header.stamp);
    updateFromOdometry(msg);
    publishEstimate(msg.header.stamp);
}

void OdometryEkf::imuCallback(const sensor_msgs::msg::Imu& msg)
{
    if (!m_is_initialized) {
        return;
    }

    predictTo(msg.header.stamp);
    updateFromImu(msg);
    publishEstimate(msg.header.stamp);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryEkf>("odometry_ekf"));
    rclcpp::shutdown();

    return 0;
}
