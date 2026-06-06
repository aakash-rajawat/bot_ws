#ifndef ODOMETRY_EKF_HPP
#define ODOMETRY_EKF_HPP

#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

class OdometryEkf : public rclcpp::Node
{
public:
    explicit OdometryEkf(const std::string& name);

private:
    using Vector5d = Eigen::Matrix<double, 5, 1>;
    using Matrix5d = Eigen::Matrix<double, 5, 5>;
    using Vector2d = Eigen::Matrix<double, 2, 1>;
    using Matrix2d = Eigen::Matrix2d;
    using Matrix2x5d = Eigen::Matrix<double, 2, 5>;
    using Vector1d = Eigen::Matrix<double, 1, 1>;
    using Matrix1x5d = Eigen::Matrix<double, 1, 5>;
    using Matrix1d = Eigen::Matrix<double, 1, 1>;

    static double normalizeAngle(double angle);
    static double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);

    void odomCallback(const nav_msgs::msg::Odometry& msg);
    void imuCallback(const sensor_msgs::msg::Imu& msg);

    void initializeFromOdometry(const nav_msgs::msg::Odometry& msg);
    void predictTo(const rclcpp::Time& stamp);

    void updateFromOdometry(const nav_msgs::msg::Odometry& msg);
    void updateFromImu(const sensor_msgs::msg::Imu& msg);

    Matrix5d buildOdometryMeasurementCovariance(const nav_msgs::msg::Odometry& msg) const;
    double getImuYawVariance(const sensor_msgs::msg::Imu& msg) const;
    double getImuYawRateVariance(const sensor_msgs::msg::Imu& msg) const;

    void publishEstimate(const rclcpp::Time& stamp);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub {};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub {};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub {};

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster {};
    geometry_msgs::msg::TransformStamped m_tf_msg {};
    nav_msgs::msg::Odometry m_odom_msg {};

    Vector5d m_state {Vector5d::Zero()};      // [x, y, yaw, v, omega]
    Matrix5d m_covariance {Matrix5d::Zero()};
    Matrix5d m_process_noise {Matrix5d::Zero()};

    rclcpp::Time m_last_predict_time {};
    bool m_is_initialized {false};

    std::string m_output_frame {"odom"};
    std::string m_output_child_frame {"base_footprint_ekf"};

    bool m_use_imu_orientation {true};
    bool m_use_imu_yaw_rate {true};

    double m_min_variance {1.0e-9};
    double m_unused_pose_variance {1.0e6};
    double m_unused_twist_variance {1.0e6};

    double m_imu_yaw_variance_override {0.05};
    double m_imu_yaw_rate_variance_override {1.0e-5};
};

#endif
