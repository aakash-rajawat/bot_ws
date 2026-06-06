#include "bot_multisensor_odometry/relative_pose_odometry.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

#include <cmath>

namespace
{
    using Vec3 = Eigen::Matrix<double, 3, 1>;
    using Mat3 = Eigen::Matrix<double, 3, 3>;
    using Cov6 = Eigen::Matrix<double, 6, 6>;

    Mat3 skewSymmetric(const Vec3& v)
    {
        Mat3 S;
        S << 0.0, -v.z(), v.y(),
             v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;
        return S;
    }

    Cov6 adjointSE3LeftPerturbation(const tf2::Transform& T)
    {
        const tf2::Matrix3x3 R_tf = T.getBasis();
        const tf2::Vector3 t_tf = T.getOrigin();

        Mat3 R;
        for(int row = 0; row < 3; ++row)
        {
            for(int col = 0; col < 3; ++col)
            {
                R(row, col) = R_tf[row][col];
            }
        }

        const Vec3 t(t_tf.x(), t_tf.y(), t_tf.z());

        Cov6 Ad = Cov6::Zero();
        Ad.topLeftCorner<3, 3>() = R;
        Ad.topRightCorner<3, 3>() = skewSymmetric(t) * R;
        Ad.bottomRightCorner<3, 3>() = R;
        return Ad;
    }

    Cov6 symmetrized(const Cov6& M)
    {
        return 0.5 * (M + M.transpose());
    }

    Cov6 rosCovToEigen(const std::array<double, 36>& cov)
    {
        Cov6 out;
        for(int r = 0; r < 6; ++r)
        {
            for(int c = 0; c < 6; ++c)
            {
                out(r, c) = cov[6 * r + c];
            }
        }
        return out;
    }

    std::array<double, 36> eigenCovToRos(const Cov6& cov)
    {
        std::array<double, 36> out {};
        for(int r = 0; r < 6; ++r)
        {
            for(int c = 0; c < 6; ++c)
            {
                out[6 * r + c] = cov(r, c);
            }
        }
        return out;
    }

    bool isValidCovariance(const Cov6& Q)
    {
        return Q.allFinite() && Q.trace() > 0.0;
    }

    geometry_msgs::msg::Twist computeTwistFromDelta(
        const tf2::Transform& T_delta,
        double dt_sec
    )
    {
        geometry_msgs::msg::Twist twist {};

        if(dt_sec <= 0.0)
        {
            return twist;
        }

        const tf2::Vector3 translation = T_delta.getOrigin();
        twist.linear.x = translation.x() / dt_sec;
        twist.linear.y = translation.y() / dt_sec;
        twist.linear.z = translation.z() / dt_sec;

        tf2::Quaternion q_delta = T_delta.getRotation();
        q_delta.normalize();

        const double angle = q_delta.getAngle();
        const tf2::Vector3 axis = q_delta.getAxis();

        if(std::isfinite(angle) && std::isfinite(axis.x()) &&
        std::isfinite(axis.y()) && std::isfinite(axis.z()) &&
        angle > 1.0e-12)
        {
            const double angular_speed = angle / dt_sec;
            twist.angular.x = axis.x() * angular_speed;
            twist.angular.y = axis.y() * angular_speed;
            twist.angular.z = axis.z() * angular_speed;
        }

        return twist;
    }
}

namespace bot_multisensor_odometry::relative_pose_odometry
{
    RelativePoseOdometry::RelativePoseOdometry(const std::string& name)
        : Node(name)
    {
        declare_parameter<std::string>("mle_pose_topic", {"/bot_controller/relative_pose_mle_lidar"});
        m_mle_pose_topic_name = get_parameter("mle_pose_topic").as_string();

        declare_parameter<std::string>("odom_mle_topic", {"/bot_controller/odom_mle_lidar"});
        m_odom_pub_topic_name = get_parameter("odom_mle_topic").as_string();

        rclcpp::QoS mle_pose_sub_qos(10);
        m_mle_pose_sub = create_subscription<bot_interfaces::msg::RelativePoseWithCovarianceStamped>(
            m_mle_pose_topic_name,
            mle_pose_sub_qos,
            std::bind(
                &RelativePoseOdometry::poseStampedCallback,
                this,
                std::placeholders::_1
            )
        );

        rclcpp::QoS odom_pub_qos(10);
        m_odom_pub = create_publisher<nav_msgs::msg::Odometry>(
            m_odom_pub_topic_name,
            odom_pub_qos
        );

        m_odom_msg.header.frame_id = "odom";
        m_odom_msg.child_frame_id = "base_footprint";
        m_odom_msg.pose.pose.orientation.w = 1.0;
    }

    void RelativePoseOdometry::poseStampedCallback(
        const bot_interfaces::msg::RelativePoseWithCovarianceStamped& stamped_pose_msg
    )
    {
        const rclcpp::Time from_stamp {stamped_pose_msg.from_stamp};
        const rclcpp::Time current_stamp {stamped_pose_msg.to_stamp};
        m_odom_msg.header.stamp = current_stamp;
        tf2::Transform T_prev {};
        tf2::Transform T_delta {};
        tf2::Transform T_curr {};

        tf2::fromMsg(m_odom_msg.pose.pose, T_prev);
        tf2::fromMsg(stamped_pose_msg.pose.pose, T_delta);

        T_curr = T_prev * T_delta;

        tf2::toMsg(T_curr, m_odom_msg.pose.pose);

        const Cov6 Q_delta = rosCovToEigen(stamped_pose_msg.pose.covariance);
        if(isValidCovariance(Q_delta))
        {
            if(!m_has_pose_covariance)
            {
                m_pose_covariance = symmetrized(Q_delta);
                m_has_pose_covariance = true;
            }
            else
            {
                const Cov6 Ad_prev = adjointSE3LeftPerturbation(T_prev);
                m_pose_covariance = symmetrized(
                    m_pose_covariance + Ad_prev * Q_delta * Ad_prev.transpose()
                );
            }
            m_odom_msg.pose.covariance = eigenCovToRos(m_pose_covariance);
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting relative pose covariance update due to invalid covariance."
            );
        }

        const double dt_sec = (current_stamp - from_stamp).seconds();
        m_odom_msg.twist.twist = computeTwistFromDelta(T_delta, dt_sec);

        m_odom_pub->publish(m_odom_msg);
        m_prev_stamp = current_stamp;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<
        bot_multisensor_odometry::relative_pose_odometry::RelativePoseOdometry
    >("relative_pose_odometry"));
    rclcpp::shutdown();
    return 0;
}
