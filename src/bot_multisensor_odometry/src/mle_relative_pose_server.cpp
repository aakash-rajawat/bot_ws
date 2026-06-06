#include "bot_multisensor_odometry/mle_relative_pose_server.hpp"
#include "bot_dugma/types.hpp"
#include "bot_dugma/dugma_registrar.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/types.hpp>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>


namespace bot_multisensor_odometry::mle_relative_pose_server
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
            double mean_x {0.0};
            double mean_y {0.0};
            double mean_z {0.0};
            double scale {0.0};
        };

        CloudDiagnostics analyzeCloud(
            const bot_interfaces::msg::PointWithCovarianceArray& cloud
        )
        {
            CloudDiagnostics d {};
            d.point_count = cloud.x.size();

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
                d.mean_x += x;
                d.mean_y += y;
                d.mean_z += z;

                Eigen::Matrix3d S {};
                S <<
                    cloud.covariance_xx[i], cloud.covariance_xy[i], cloud.covariance_xz[i],
                    cloud.covariance_xy[i], cloud.covariance_yy[i], cloud.covariance_yz[i],
                    cloud.covariance_xz[i], cloud.covariance_yz[i], cloud.covariance_zz[i];

                if(!S.allFinite())
                {
                    ++d.nonfinite_covariances;
                    continue;
                }

                if(!S.isApprox(S.transpose(), 1e-9))
                {
                    ++d.nonsymmetric_covariances;
                }

                const double det = S.determinant();
                d.min_det = std::min(d.min_det, det);
                if(det <= 0.0)
                {
                    ++d.nonpositive_det_covariances;
                }

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_solver(S);
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
                d.mean_x /= static_cast<double>(d.valid_xyz_count);
                d.mean_y /= static_cast<double>(d.valid_xyz_count);
                d.mean_z /= static_cast<double>(d.valid_xyz_count);

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

                    const double dx = x - d.mean_x;
                    const double dy = y - d.mean_y;
                    const double dz = z - d.mean_z;
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


        Eigen::Matrix3d extractPlanarCovarianceFromRos(
            const std::array<double, 36>& cov
        )
        {
            Eigen::Matrix3d Q;
            Q << cov[0], cov[1], cov[5],
                 cov[6], cov[7], cov[11],
                 cov[30], cov[31], cov[35];
            return Q;
        }

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
    // to be removed later: end

    MLERelativePoseServer::MLERelativePoseServer(const rclcpp::NodeOptions& options)
        : Node("mle_relative_pose_server", options)
    {
        declare_parameter<std::string>(
            "relative_pose_action_name",
            {"/mle_relative_pose_lidar"}
        );
        m_relative_pose_action_name = get_parameter("relative_pose_action_name").as_string();

        m_relative_pose_server =
            rclcpp_action::create_server<bot_interfaces::action::MLERelativePose>(
            this,
            m_relative_pose_action_name,
            std::bind(
                &MLERelativePoseServer::goalCallback,
                this, 
                std::placeholders::_1, 
                std::placeholders::_2
            ),
            std::bind(
                &MLERelativePoseServer::cancelCallback,
                this,
                std::placeholders::_1
            ),
            std::bind(
                &MLERelativePoseServer::acceptedCallback,
                this,
                std::placeholders::_1
            )
        );

        m_worker_thread = std::thread(
            &MLERelativePoseServer::workerLoop,
            this
        );

        RCLCPP_INFO(get_logger(), "MLE relative pose server is up and running...");
    }

    rclcpp_action::GoalResponse MLERelativePoseServer::goalCallback(
        const rclcpp_action::GoalUUID& goal_uuid,
        std::shared_ptr<
            const bot_interfaces::action::MLERelativePose::Goal
        > goal
    )
    {
        RCLCPP_INFO_STREAM(get_logger(), "Received fused point clouds X0 and Y0 with Goal UUID: "
                                            << rclcpp_action::to_string(goal_uuid));

        
        std::size_t N = goal->x0.x.size();
        if(N == 0)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Input point cloud: X0 is empty. "
                                                << "Rejecting Goal with UUID: " 
                                                << rclcpp_action::to_string(goal_uuid));
            return rclcpp_action::GoalResponse::REJECT;
        }

        std::size_t M = goal->y0.x.size();
        if(M == 0)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Input point cloud: Y0 is empty. "
                                                << "Rejecting Goal with UUID: " 
                                                << rclcpp_action::to_string(goal_uuid));
            return rclcpp_action::GoalResponse::REJECT;
        }

        if(
            goal->x0.y.size() != N
            || goal->x0.z.size() != N
            || goal->x0.covariance_xx.size() != N
            || goal->x0.covariance_xy.size() != N
            || goal->x0.covariance_xz.size() != N
            || goal->x0.covariance_yy.size() != N
            || goal->x0.covariance_yz.size() != N
            || goal->x0.covariance_zz.size() != N        
        )
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Mal-formed input point cloud: X0 "
                                                << "Rejecting Goal with UUID: " 
                                                << rclcpp_action::to_string(goal_uuid));
            return rclcpp_action::GoalResponse::REJECT;
        }

        
        if(
            goal->y0.y.size() != M
            || goal->y0.z.size() != M
            || goal->y0.covariance_xx.size() != M
            || goal->y0.covariance_xy.size() != M
            || goal->y0.covariance_xz.size() != M
            || goal->y0.covariance_yy.size() != M
            || goal->y0.covariance_yz.size() != M
            || goal->y0.covariance_zz.size() != M        
        )
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Mal-formed input point cloud: Y0 "
                                                << "Rejecting Goal with UUID: " 
                                                << rclcpp_action::to_string(goal_uuid));
            return rclcpp_action::GoalResponse::REJECT;
        }

        std::lock_guard<std::mutex> lock(m_def_goal_handles_mutex);
        if(m_def_goal_handles.size() >= m_max_def_goals)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Exceeding the limit of accepted goals. "
                                                << "Rejecting Goal with UUID: " 
                                                << rclcpp_action::to_string(goal_uuid));
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO_STREAM(get_logger(), "Accepting Point Clouds with Goal UUID: "
                                            << rclcpp_action::to_string(goal_uuid));
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    }

    rclcpp_action::CancelResponse MLERelativePoseServer::cancelCallback(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<bot_interfaces::action::MLERelativePose>
        > goal_handle
    )
    {
        (void)goal_handle;
        RCLCPP_INFO(get_logger(), "Received request from MLE relative pose client to cancel goal. Canceling...");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void MLERelativePoseServer::acceptedCallback(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<bot_interfaces::action::MLERelativePose>
        > goal_handle
    )
    {
        {
            std::lock_guard<std::mutex> lock(m_def_goal_handles_mutex);
            m_def_goal_handles.push_back(goal_handle);
        }

        m_def_goal_handles_cond_var.notify_one();
        RCLCPP_INFO(get_logger(), "MLE relative pose goal deferred.");
    }

    void MLERelativePoseServer::workerLoop()
    {
        while(rclcpp::ok())
        {
            std::shared_ptr<
                rclcpp_action::ServerGoalHandle<bot_interfaces::action::MLERelativePose>
            > goal_handle {};

            {
                std::unique_lock<std::mutex> lock(m_def_goal_handles_mutex);

                m_def_goal_handles_cond_var.wait(
                    lock,
                    [this]()
                    {
                        return m_stop_worker || !m_def_goal_handles.empty();
                    }
                );

                if(m_stop_worker)
                {
                    return;
                }

                goal_handle = m_def_goal_handles.front();
                m_def_goal_handles.pop_front();
            }

            if(goal_handle->is_canceling())
            {
                auto result = std::make_shared<bot_interfaces::action::MLERelativePose::Result>();
                result->converged = false;
                result->message = "Goal cancelled before execution.";
                goal_handle->canceled(result);
                continue;
            }

            goal_handle->execute();
            try
            {
                estimatePose(goal_handle);
            }
            catch(const std::exception& e)
            {
                auto result = std::make_shared<bot_interfaces::action::MLERelativePose::Result>();
                const auto goal = goal_handle->get_goal();
                result->converged = false;
                result->relative_pose.header = goal->y0.header;
                result->relative_pose.from_stamp = goal->x0.header.stamp;
                result->relative_pose.to_stamp = goal->y0.header.stamp;
                result->relative_pose.from_frame_id = goal->x0.header.frame_id;
                result->relative_pose.to_frame_id = goal->y0.header.frame_id;
                result->message = std::string("DUGMA execution threw exception: ") + e.what();

                RCLCPP_ERROR(
                    get_logger(),
                    "DUGMA action ABORTED due to exception: message='%s' "
                    "x_points=%zu y_points=%zu x_stamp=%.6f y_stamp=%.6f",
                    result->message.c_str(),
                    goal->x0.x.size(),
                    goal->y0.x.size(),
                    rclcpp::Time(goal->x0.header.stamp).seconds(),
                    rclcpp::Time(goal->y0.header.stamp).seconds()
                );

                goal_handle->abort(result);
            }
            catch(...)
            {
                auto result = std::make_shared<bot_interfaces::action::MLERelativePose::Result>();
                const auto goal = goal_handle->get_goal();
                result->converged = false;
                result->relative_pose.header = goal->y0.header;
                result->relative_pose.from_stamp = goal->x0.header.stamp;
                result->relative_pose.to_stamp = goal->y0.header.stamp;
                result->relative_pose.from_frame_id = goal->x0.header.frame_id;
                result->relative_pose.to_frame_id = goal->y0.header.frame_id;
                result->message = "DUGMA execution threw an unknown exception.";

                RCLCPP_ERROR(
                    get_logger(),
                    "DUGMA action ABORTED due to unknown exception: message='%s' "
                    "x_points=%zu y_points=%zu x_stamp=%.6f y_stamp=%.6f",
                    result->message.c_str(),
                    goal->x0.x.size(),
                    goal->y0.x.size(),
                    rclcpp::Time(goal->x0.header.stamp).seconds(),
                    rclcpp::Time(goal->y0.header.stamp).seconds()
                );

                goal_handle->abort(result);
            }
        }
    }

    void MLERelativePoseServer::estimatePose(
        const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<bot_interfaces::action::MLERelativePose>
        > goal_handle
    )
    {
        const auto goal = goal_handle->get_goal();

        // to be removed later: begin
        logCloudDiagnostics(get_logger(), "X0 diagnostics", goal->x0);
        logCloudDiagnostics(get_logger(), "Y0 diagnostics", goal->y0);
        // to be removed later: end

        bot_dugma::PointCloudView in_X {};
        in_X.m_x = goal->x0.x;
        in_X.m_y = goal->x0.y;
        in_X.m_z = goal->x0.z;
        in_X.m_Sxx = goal->x0.covariance_xx;
        in_X.m_Sxy = goal->x0.covariance_xy;
        in_X.m_Sxz = goal->x0.covariance_xz;
        in_X.m_Syy = goal->x0.covariance_yy;
        in_X.m_Syz = goal->x0.covariance_yz;
        in_X.m_Szz = goal->x0.covariance_zz;
        in_X.m_frame_id = goal->x0.header.frame_id;
        in_X.m_timestamp_sec = rclcpp::Time(goal->x0.header.stamp).seconds();

        bot_dugma::PointCloudView in_Y {};
        in_Y.m_x = goal->y0.x;
        in_Y.m_y = goal->y0.y;
        in_Y.m_z = goal->y0.z;
        in_Y.m_Sxx = goal->y0.covariance_xx; 
        in_Y.m_Sxy = goal->y0.covariance_xy;
        in_Y.m_Sxz = goal->y0.covariance_xz;
        in_Y.m_Syy = goal->y0.covariance_yy;
        in_Y.m_Syz = goal->y0.covariance_yz;
        in_Y.m_Szz = goal->y0.covariance_zz;
        in_Y.m_frame_id = goal->y0.header.frame_id;
        in_Y.m_timestamp_sec = rclcpp::Time(goal->y0.header.stamp).seconds();

        bot_dugma::DugmaRegistrar dugma_registrar {m_registrar_config};
        bot_dugma::PoseEstimate initial_guess_pose {};
        const bot_dugma::PoseEstimate* init_guess = nullptr;
        if(goal->has_initial_guess)
        {
            initial_guess_pose.m_t[0] =
                static_cast<float>(goal->initial_guess.position.x);
            initial_guess_pose.m_t[1] =
                static_cast<float>(goal->initial_guess.position.y);
            initial_guess_pose.m_t[2] =
                static_cast<float>(goal->initial_guess.position.z);

            const auto& q_msg = goal->initial_guess.orientation;
            Eigen::Quaternionf q(
                static_cast<float>(q_msg.w),
                static_cast<float>(q_msg.x),
                static_cast<float>(q_msg.y),
                static_cast<float>(q_msg.z)
            );
            q.normalize();
            Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(
                initial_guess_pose.m_R
            ) = q.toRotationMatrix();
            init_guess = &initial_guess_pose;
        }
        bot_dugma::PoseEstimate curr_pose_estimate = 
            dugma_registrar.estimate(in_X, in_Y, init_guess);

        completeGoal(curr_pose_estimate, goal_handle);
    }

    void MLERelativePoseServer::completeGoal(
        const bot_dugma::PoseEstimate& curr_pose_estimate,
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<bot_interfaces::action::MLERelativePose>
        > goal_handle
    )
    {
        auto result = std::make_shared<bot_interfaces::action::MLERelativePose::Result>();
        const auto goal = goal_handle->get_goal();

        // DUGMA registers y0 into x0:
        //     x0 ~= T_x0_y0 * y0
        // Therefore this result is T_from_to with from=x0 and to=y0.
        result->relative_pose.header = goal->y0.header;
        result->relative_pose.header.stamp = goal->y0.header.stamp;
        result->relative_pose.from_stamp = goal->x0.header.stamp;
        result->relative_pose.to_stamp = goal->y0.header.stamp;
        result->relative_pose.from_frame_id = goal->x0.header.frame_id;
        result->relative_pose.to_frame_id = goal->y0.header.frame_id;
        result->converged = curr_pose_estimate.m_is_converged;

        RCLCPP_INFO(
            get_logger(),
            "DUGMA raw result: converged=%s iter=%d energy=%.9e "
            "t=(%.6f, %.6f, %.6f) "
            "has_previous_estimate=%s "
            "x_points=%zu y_points=%zu "
            "x_stamp=%.6f y_stamp=%.6f",
            curr_pose_estimate.m_is_converged ? "true" : "false",
            curr_pose_estimate.m_iter,
            static_cast<double>(curr_pose_estimate.m_energy),
            curr_pose_estimate.m_t[0],
            curr_pose_estimate.m_t[1],
            curr_pose_estimate.m_t[2],
            goal->has_initial_guess ? "true" : "false",
            goal->x0.x.size(),
            goal->y0.x.size(),
            rclcpp::Time(goal->x0.header.stamp).seconds(),
            rclcpp::Time(goal->y0.header.stamp).seconds()
        );

        if(curr_pose_estimate.m_is_converged)
        {
            if(!curr_pose_estimate.m_has_pose_cov)
            {
                result->message =
                    "DUGMA converged, but pose covariance estimation failed. Aborting result.";
                result->converged = false;

                RCLCPP_ERROR(
                    get_logger(),
                    "%s",
                    result->message.c_str()
                );
                goal_handle->abort(result);
                return;
            }

            result->message = "DUGMA algorithm converged..";
            result->relative_pose.pose.pose.position.x = curr_pose_estimate.m_t[0];
            result->relative_pose.pose.pose.position.y = curr_pose_estimate.m_t[1];
            result->relative_pose.pose.pose.position.z = curr_pose_estimate.m_t[2];

            Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> R(
                curr_pose_estimate.m_R
            );
            Eigen::Quaternionf q(R);
            q.normalize();
            result->relative_pose.pose.pose.orientation.w = q.w();
            result->relative_pose.pose.pose.orientation.x = q.x();
            result->relative_pose.pose.pose.orientation.y = q.y();
            result->relative_pose.pose.pose.orientation.z = q.z();

            std::fill(
                result->relative_pose.pose.covariance.begin(),
                result->relative_pose.pose.covariance.end(),
                0.0
            );

            const auto& c = curr_pose_estimate.m_pose_cov;

            result->relative_pose.pose.covariance[0]  = c[0];
            result->relative_pose.pose.covariance[1]  = c[1];
            result->relative_pose.pose.covariance[2]  = c[2];
            result->relative_pose.pose.covariance[3]  = c[3];
            result->relative_pose.pose.covariance[4]  = c[4];
            result->relative_pose.pose.covariance[5]  = c[5];

            result->relative_pose.pose.covariance[6]  = c[1];
            result->relative_pose.pose.covariance[7]  = c[6];
            result->relative_pose.pose.covariance[8]  = c[7];
            result->relative_pose.pose.covariance[9]  = c[8];
            result->relative_pose.pose.covariance[10] = c[9];
            result->relative_pose.pose.covariance[11] = c[10];

            result->relative_pose.pose.covariance[12] = c[2];
            result->relative_pose.pose.covariance[13] = c[7];
            result->relative_pose.pose.covariance[14] = c[11];
            result->relative_pose.pose.covariance[15] = c[12];
            result->relative_pose.pose.covariance[16] = c[13];
            result->relative_pose.pose.covariance[17] = c[14];

            result->relative_pose.pose.covariance[18] = c[3];
            result->relative_pose.pose.covariance[19] = c[8];
            result->relative_pose.pose.covariance[20] = c[12];
            result->relative_pose.pose.covariance[21] = c[15];
            result->relative_pose.pose.covariance[22] = c[16];
            result->relative_pose.pose.covariance[23] = c[17];

            result->relative_pose.pose.covariance[24] = c[4];
            result->relative_pose.pose.covariance[25] = c[9];
            result->relative_pose.pose.covariance[26] = c[13];
            result->relative_pose.pose.covariance[27] = c[16];
            result->relative_pose.pose.covariance[28] = c[18];
            result->relative_pose.pose.covariance[29] = c[19];

            result->relative_pose.pose.covariance[30] = c[5];
            result->relative_pose.pose.covariance[31] = c[10];
            result->relative_pose.pose.covariance[32] = c[14];
            result->relative_pose.pose.covariance[33] = c[17];
            result->relative_pose.pose.covariance[34] = c[19];
            result->relative_pose.pose.covariance[35] = c[20];

            const Eigen::Matrix3d Q_planar =
                extractPlanarCovarianceFromRos(result->relative_pose.pose.covariance);
            if(!isValidCovariance3(Q_planar))
            {
                result->message =
                    "DUGMA covariance is invalid after packing. Aborting result.";
                result->converged = false;

                RCLCPP_ERROR(
                    get_logger(),
                    "%s",
                    result->message.c_str()
                );
                goal_handle->abort(result);
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "DUGMA action SUCCEEDED with current estimate: "
                "message='%s' position=(%.6f, %.6f, %.6f) "
                "orientation_xyzw=(%.6f, %.6f, %.6f, %.6f)",
                result->message.c_str(),
                result->relative_pose.pose.pose.position.x,
                result->relative_pose.pose.pose.position.y,
                result->relative_pose.pose.pose.position.z,
                result->relative_pose.pose.pose.orientation.x,
                result->relative_pose.pose.pose.orientation.y,
                result->relative_pose.pose.pose.orientation.z,
                result->relative_pose.pose.pose.orientation.w
            );

            goal_handle->succeed(result);
        }
        else if(goal->has_initial_guess)
        {
            result->message =
                "DUGMA did not converge. Not returning the initial guess as a successful measurement.";
            result->converged = false;

            RCLCPP_WARN(
                get_logger(),
                "%s failed_iter=%d failed_energy=%.9e",
                result->message.c_str(),
                curr_pose_estimate.m_iter,
                static_cast<double>(curr_pose_estimate.m_energy)
            );

            goal_handle->abort(result);
            return;
        }
        else
        {
            result->message = "DUGMA algorithm did not converge for these 2 point clouds.";

            RCLCPP_ERROR(
                get_logger(),
                "DUGMA action ABORTED with no previous estimate: "
                "message='%s' failed_iter=%d failed_energy=%.9e "
                "failed_t=(%.6f, %.6f, %.6f)",
                result->message.c_str(),
                curr_pose_estimate.m_iter,
                static_cast<double>(curr_pose_estimate.m_energy),
                curr_pose_estimate.m_t[0],
                curr_pose_estimate.m_t[1],
                curr_pose_estimate.m_t[2]
            );

            goal_handle->abort(result);
        }
    }

    MLERelativePoseServer::~MLERelativePoseServer()
    {
        {
            std::lock_guard<std::mutex> lock(m_def_goal_handles_mutex);
            m_stop_worker = true;
        }

        m_def_goal_handles_cond_var.notify_all();

        if(m_worker_thread.joinable())
        {
            m_worker_thread.join();
        }
    }
}


RCLCPP_COMPONENTS_REGISTER_NODE(
    bot_multisensor_odometry::mle_relative_pose_server::MLERelativePoseServer
);
