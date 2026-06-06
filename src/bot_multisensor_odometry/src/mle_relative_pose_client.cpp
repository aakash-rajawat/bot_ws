#include "bot_multisensor_odometry/mle_relative_pose_client.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace bot_multisensor_odometry::mle_relative_pose_client
{
    // to be removed later: begin
    namespace
    {
        struct CloudDiagnostics
        {
            std::size_t point_count {0};
            std::size_t valid_xyz_count {0};
            std::size_t nonfinite_points {0};
            std::size_t nonfinite_covariances {0};
            std::size_t nonsymmetric_covariances {0};
            std::size_t nonpositive_det_covariances {0};
            double min_det {std::numeric_limits<double>::infinity()};
            double min_eig {std::numeric_limits<double>::infinity()};
            double min_x {std::numeric_limits<double>::infinity()};
            double min_y {std::numeric_limits<double>::infinity()};
            double min_z {std::numeric_limits<double>::infinity()};
            double max_x {-std::numeric_limits<double>::infinity()};
            double max_y {-std::numeric_limits<double>::infinity()};
            double max_z {-std::numeric_limits<double>::infinity()};
            double scale {0.0};
        };

        CloudDiagnostics analyzeCloud(
            const bot_interfaces::msg::PointWithCovarianceArray& cloud
        )
        {
            CloudDiagnostics d {};
            d.point_count = cloud.x.size();

            double sum_x = 0.0;
            double sum_y = 0.0;
            double sum_z = 0.0;

            for(std::size_t i = 0; i < d.point_count; ++i)
            {
                const double x = cloud.x[i];
                const double y = cloud.y[i];
                const double z = cloud.z[i];

                if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                {
                    ++d.nonfinite_points;
                    continue;
                }

                ++d.valid_xyz_count;
                d.min_x = std::min(d.min_x, x);
                d.min_y = std::min(d.min_y, y);
                d.min_z = std::min(d.min_z, z);
                d.max_x = std::max(d.max_x, x);
                d.max_y = std::max(d.max_y, y);
                d.max_z = std::max(d.max_z, z);
                sum_x += x;
                sum_y += y;
                sum_z += z;

                Eigen::Matrix3d sigma;
                sigma <<
                    cloud.covariance_xx[i], cloud.covariance_xy[i], cloud.covariance_xz[i],
                    cloud.covariance_xy[i], cloud.covariance_yy[i], cloud.covariance_yz[i],
                    cloud.covariance_xz[i], cloud.covariance_yz[i], cloud.covariance_zz[i];

                if(!sigma.allFinite())
                {
                    ++d.nonfinite_covariances;
                    continue;
                }

                if(!sigma.isApprox(sigma.transpose(), 1e-9))
                {
                    ++d.nonsymmetric_covariances;
                }

                const double det = sigma.determinant();
                d.min_det = std::min(d.min_det, det);
                if(det <= 0.0)
                {
                    ++d.nonpositive_det_covariances;
                }

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_solver(sigma);
                if(eig_solver.info() == Eigen::Success)
                {
                    d.min_eig = std::min(
                        d.min_eig,
                        static_cast<double>(eig_solver.eigenvalues().minCoeff())
                    );
                }
            }

            if(d.valid_xyz_count > 0)
            {
                const double mean_x = sum_x / static_cast<double>(d.valid_xyz_count);
                const double mean_y = sum_y / static_cast<double>(d.valid_xyz_count);
                const double mean_z = sum_z / static_cast<double>(d.valid_xyz_count);

                double accum = 0.0;
                for(std::size_t i = 0; i < d.point_count; ++i)
                {
                    const double x = cloud.x[i];
                    const double y = cloud.y[i];
                    const double z = cloud.z[i];

                    if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                    {
                        continue;
                    }

                    const double dx = x - mean_x;
                    const double dy = y - mean_y;
                    const double dz = z - mean_z;
                    accum += dx * dx + dy * dy + dz * dz;
                }

                d.scale = std::sqrt(accum / static_cast<double>(d.valid_xyz_count));
            }

            return d;
        }

        void logCloudDiagnostics(
            const rclcpp::Logger& logger,
            const char* label,
            const bot_interfaces::msg::PointWithCovarianceArray& cloud
        )
        {
            const CloudDiagnostics d = analyzeCloud(cloud);

            RCLCPP_WARN(
                logger,
                "%s frame=%s stamp=%.6f points=%zu valid_xyz=%zu "
                "xyz_min=(%.6f, %.6f, %.6f) xyz_max=(%.6f, %.6f, %.6f) "
                "nonfinite_points=%zu nonfinite_cov=%zu nonsymmetric_cov=%zu det_le_zero=%zu "
                "min_det=%.6e min_eig=%.6e scale=%.6e",
                label,
                cloud.header.frame_id.c_str(),
                rclcpp::Time(cloud.header.stamp).seconds(),
                d.point_count,
                d.valid_xyz_count,
                d.min_x, d.min_y, d.min_z,
                d.max_x, d.max_y, d.max_z,
                d.nonfinite_points,
                d.nonfinite_covariances,
                d.nonsymmetric_covariances,
                d.nonpositive_det_covariances,
                d.min_det,
                d.min_eig,
                d.scale
            );
        }
    }
    // to be removed later: end

    MLERelativePoseClient::MLERelativePoseClient(
        const rclcpp::NodeOptions& options
    )
    : Node("mle_relative_pose_client", options)
    {
        declare_parameter<std::string>("topic_point_cloud", {"/ua_lidar/ua_point_cloud"});
        m_topic_pt_cloud = get_parameter("topic_point_cloud").as_string();

        declare_parameter<std::string>(
            "topic_relative_pose",
            {"/bot_controller/relative_pose_mle_lidar"}
        );
        m_topic_relative_pose = get_parameter("topic_relative_pose").as_string();

        declare_parameter<std::string>(
            "relative_pose_action_name",
            {"/mle_relative_pose_lidar"}
        );
        m_relative_pose_action_name = get_parameter("relative_pose_action_name").as_string();

        declare_parameter("skip_stationary_cloud_pairs", true);
        m_skip_stationary_cloud_pairs =
            get_parameter("skip_stationary_cloud_pairs").as_bool();

        declare_parameter("stationary_translation_threshold", 0.01);
        m_stationary_translation_threshold =
            get_parameter("stationary_translation_threshold").as_double();

        declare_parameter("stationary_yaw_threshold", 0.017453292519943295);
        m_stationary_yaw_threshold = get_parameter("stationary_yaw_threshold").as_double();

        rclcpp::QoS pt_cloud_sub_qos(10);
        m_pt_cloud_sub = create_subscription<bot_interfaces::msg::PointWithCovarianceArray>(
            m_topic_pt_cloud,
            pt_cloud_sub_qos,
            std::bind(&MLERelativePoseClient::pointCloudCallback, this, std::placeholders::_1)
        );

        m_wheel_odom_sub.subscribe(
            this,
            "/wheel_odometry/pose"
        );

        m_relative_pose_client =
            rclcpp_action::create_client<bot_interfaces::action::MLERelativePose>(
            this,
            m_relative_pose_action_name
        );

        rclcpp::QoS relative_pose_pub_qos(10);
        m_relative_pose_pub = create_publisher<bot_interfaces::msg::RelativePoseWithCovarianceStamped>(
            m_topic_relative_pose,
            relative_pose_pub_qos
        );

        
    }

    void MLERelativePoseClient::pointCloudCallback(
        const bot_interfaces::msg::PointWithCovarianceArray::ConstSharedPtr pt_cloud_msg
    )
    {
        const rclcpp::Time curr_stamp {pt_cloud_msg->header.stamp};

        if(!m_previous_stamp)
        {
            m_previous_cloud = pt_cloud_msg;
            m_previous_stamp = curr_stamp;
            return;
        }

        const rclcpp::Duration time_diff =
            curr_stamp - *m_previous_stamp;

        if(time_diff <= rclcpp::Duration(0, 0))
        {
            return;
        }

        if(time_diff < m_association_tolerance)
        {
            RCLCPP_WARN(get_logger(), "Time difference between current syncedPointCloudCallback and previous too small.");
            return;
        }

        if(!m_relative_pose_client->action_server_is_ready())
        {
            RCLCPP_WARN(get_logger(), "DUGMA Action Server not yet ready.");
            return;
        }

        if(m_goal_in_flight)
        {
            m_pending_cloud = pt_cloud_msg;
            return;
        }

        bot_interfaces::msg::PointWithCovarianceArray previous_pt_cloud =
            *m_previous_cloud;

        bot_interfaces::msg::PointWithCovarianceArray current_pt_cloud =
            *pt_cloud_msg;

        // to be removed later: begin
        logCloudDiagnostics(
            get_logger(),
            "Previous cloud diagnostics",
            *m_previous_cloud
        );
        
        logCloudDiagnostics(
            get_logger(),
            "Current cloud diagnostics",
            *pt_cloud_msg
        );
        
        // to be removed later: end

        RCLCPP_INFO(
            get_logger(),
            "Accepted cloud pair dt=%.3f previous_points=%zu current_points=%zu",
            time_diff.seconds(),
            previous_pt_cloud.x.size(),
            current_pt_cloud.x.size()
        );

        m_goal_in_flight = true;
        sendGoalMessage(previous_pt_cloud, current_pt_cloud);

        m_previous_cloud = pt_cloud_msg;
        m_previous_stamp = curr_stamp;
    }

    void MLERelativePoseClient::maybeDispatchPendingCloud()
    {
        if(!m_pending_cloud)
        {
            return;
        }

        auto pending_cloud = m_pending_cloud;
        m_pending_cloud.reset();
        pointCloudCallback(pending_cloud);
    }

    geometry_msgs::msg::Pose computeRelativeTransform(
        const geometry_msgs::msg::PoseStamped& wheel_odom_X0,
        const geometry_msgs::msg::PoseStamped& wheel_odom_Y0
    )
    {
        tf2::Transform T_X0, T_Y0;
        tf2::fromMsg(wheel_odom_X0.pose, T_X0);
        tf2::fromMsg(wheel_odom_Y0.pose, T_Y0);

        geometry_msgs::msg::Pose out {};
        tf2::toMsg(T_X0.inverseTimes(T_Y0), out);
        return out;
    }

    void MLERelativePoseClient::sendGoalMessage(
        const bot_interfaces::msg::PointWithCovarianceArray& x0,
        const bot_interfaces::msg::PointWithCovarianceArray& y0
    )
    {
        // create goal message
        auto relative_pose_goal_msg = bot_interfaces::action::MLERelativePose::Goal();
        relative_pose_goal_msg.x0 = x0;
        relative_pose_goal_msg.y0 = y0;

        // form initial guess
        const rclcpp::Time t_X0 = x0.header.stamp;
        const rclcpp::Time t_Y0 = y0.header.stamp;

        // debug begins
        const auto wheel_odom_msg_X0 = m_wheel_odom_cache.getElemBeforeTime(t_X0);
        const auto wheel_odom_msg_Y0 = m_wheel_odom_cache.getElemBeforeTime(t_Y0);
        if(!wheel_odom_msg_X0 || !wheel_odom_msg_Y0)
        {
            const rclcpp::Time cache_oldest = m_wheel_odom_cache.getOldestTime();
            const rclcpp::Time cache_latest = m_wheel_odom_cache.getLatestTime();

            RCLCPP_WARN(
                get_logger(),
                "Wheel odom cache miss: has_X0=%s has_Y0=%s "
                "t_X0=%.6f t_Y0=%.6f cache_oldest=%.6f cache_latest=%.6f",
                wheel_odom_msg_X0 ? "true" : "false",
                wheel_odom_msg_Y0 ? "true" : "false",
                t_X0.seconds(),
                t_Y0.seconds(),
                cache_oldest.seconds(),
                cache_latest.seconds()
            );
            RCLCPP_WARN(
                get_logger(),
                "Skipping DUGMA goal because wheel-odometry initial guess is unavailable."
            );
            m_goal_in_flight = false;
            maybeDispatchPendingCloud();
            return;
        }
        else
        {
            const geometry_msgs::msg::Pose initial_guess = computeRelativeTransform(
                *wheel_odom_msg_X0,
                *wheel_odom_msg_Y0
            );

            tf2::Quaternion q {};
            tf2::fromMsg(initial_guess.orientation, q);

            double roll {};
            double pitch {};
            double yaw {};
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            const double planar_translation = std::hypot(
                initial_guess.position.x,
                initial_guess.position.y
            );

            if(
                m_skip_stationary_cloud_pairs
                && planar_translation < m_stationary_translation_threshold
                && std::abs(yaw) < m_stationary_yaw_threshold
            )
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Skipping DUGMA goal because positional and yaw increments are too low: "
                    "translation=%.6f m yaw=%.6f rad thresholds=(%.6f m, %.6f rad).",
                    planar_translation,
                    yaw,
                    m_stationary_translation_threshold,
                    m_stationary_yaw_threshold
                );
                m_goal_in_flight = false;
                maybeDispatchPendingCloud();
                return;
            }

            relative_pose_goal_msg.has_initial_guess = true;
            relative_pose_goal_msg.initial_guess = initial_guess;
        }

    
        // define SendGoalOptions object
        auto send_goal_options = rclcpp_action::Client<
                bot_interfaces::action::MLERelativePose
            >::SendGoalOptions();

        send_goal_options.goal_response_callback = std::bind(
            &MLERelativePoseClient::goalCallback,
            this,
            std::placeholders::_1
        );

        send_goal_options.feedback_callback = std::bind(
            &MLERelativePoseClient::feedbackCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        );

        send_goal_options.result_callback = std::bind(
            &MLERelativePoseClient::resultCallback,
            this,
            std::placeholders::_1
        );
        // send goal objects to server
        m_relative_pose_client->async_send_goal(relative_pose_goal_msg, send_goal_options);
    }

    void MLERelativePoseClient::goalCallback(
        const rclcpp_action::ClientGoalHandle<
            bot_interfaces::action::MLERelativePose
        >::SharedPtr& goal_handle
    )
    {
        if(goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Fused Point Clouds accepted by DUGMA Server.");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Fused Point Clouds rejected by DUGMA Server.");
            m_goal_in_flight = false;
            maybeDispatchPendingCloud();
        }
    }

    void MLERelativePoseClient::feedbackCallback(
        rclcpp_action::ClientGoalHandle<
            bot_interfaces::action::MLERelativePose
        >::SharedPtr,
        const std::shared_ptr<
            const bot_interfaces::action::MLERelativePose::Feedback
        > feedback
    )
    {
        RCLCPP_INFO_STREAM(get_logger(), feedback->stage);
    }

    void MLERelativePoseClient::resultCallback(
        rclcpp_action::ClientGoalHandle<
            bot_interfaces::action::MLERelativePose
        >::WrappedResult result
    )
    {
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(
                    get_logger(),
                    "DUGMA action result ABORTED: converged=%s message='%s'",
                    result.result ? (result.result->converged ? "true" : "false") : "unknown",
                    result.result ? result.result->message.c_str() : "<no result>"
                );
                m_goal_in_flight = false;
                maybeDispatchPendingCloud();
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(
                    get_logger(),
                    "DUGMA action result CANCELED: converged=%s message='%s'",
                    result.result ? (result.result->converged ? "true" : "false") : "unknown",
                    result.result ? result.result->message.c_str() : "<no result>"
                );
                m_goal_in_flight = false;
                maybeDispatchPendingCloud();
                return;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown result code.");
                m_goal_in_flight = false;
                maybeDispatchPendingCloud();
                return;
        }

        if(!result.result)
        {
            RCLCPP_ERROR(get_logger(), "DUGMA action succeeded but result is null.");
            m_goal_in_flight = false;
            maybeDispatchPendingCloud();
            return;
        }

        if(!result.result->converged)
        {
            RCLCPP_WARN(
                get_logger(),
                "DUGMA result is not converged. Not publishing relative-pose factor."
            );
            m_goal_in_flight = false;
            maybeDispatchPendingCloud();
            return;
        }

        const auto& relative_pose = result.result->relative_pose;
        if(relative_pose.from_stamp.sec == 0 && relative_pose.from_stamp.nanosec == 0)
        {
            RCLCPP_WARN(get_logger(), "DUGMA relative pose has empty from_stamp. Not publishing.");
            m_goal_in_flight = false;
            maybeDispatchPendingCloud();
            return;
        }

        if(relative_pose.to_stamp.sec == 0 && relative_pose.to_stamp.nanosec == 0)
        {
            RCLCPP_WARN(get_logger(), "DUGMA relative pose has empty to_stamp. Not publishing.");
            m_goal_in_flight = false;
            maybeDispatchPendingCloud();
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "DUGMA action result SUCCEEDED: converged=%s message='%s' "
            "position=(%.6f, %.6f, %.6f) "
            "orientation_xyzw=(%.6f, %.6f, %.6f, %.6f)",
            result.result->converged ? "true" : "false",
            result.result->message.c_str(),
            relative_pose.pose.pose.position.x,
            relative_pose.pose.pose.position.y,
            relative_pose.pose.pose.position.z,
            relative_pose.pose.pose.orientation.x,
            relative_pose.pose.pose.orientation.y,
            relative_pose.pose.pose.orientation.z,
            relative_pose.pose.pose.orientation.w
        );

        m_relative_pose_pub->publish(relative_pose);
        m_goal_in_flight = false;
        maybeDispatchPendingCloud();
    }

}


RCLCPP_COMPONENTS_REGISTER_NODE(
    bot_multisensor_odometry::mle_relative_pose_client::MLERelativePoseClient
);
