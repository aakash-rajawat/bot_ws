#include "bot_vision/triangulation.hpp"
#include "bot_vision/casADi_helper.hpp"

#include <vector>

namespace bot_vision::triangulation
{
    bool finitenessCheck(const Eigen::Vector3d& X_i, const Eigen::Matrix3d& Sigma_X_i)
    {
        return (X_i.allFinite() && Sigma_X_i.allFinite());
    }

    bool cheiralityCheck(const Eigen::Vector3d& X_i)
    {
        return (X_i.z() > 0.0);
    }

    bool symPosSemiDefCheck(
        const Eigen::Matrix3d& Sigma_X_i, 
        const double symmetric_tolerance, 
        const double positive_eigenvalue_tolerance
    )
    {
        const bool symmetric =
            Sigma_X_i.isApprox(Sigma_X_i.transpose(), symmetric_tolerance);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_solver(Sigma_X_i);
        const bool eig_ok = (eig_solver.info() == Eigen::Success);

        const bool positive_semidefinite =
            eig_ok && (eig_solver.eigenvalues().array() >= -positive_eigenvalue_tolerance).all();

        return symmetric && positive_semidefinite;
    }

    bool reprojectionErrorCheck(
        const Eigen::Vector3d& X_i,
        const std::size_t pt_id,
        const std::vector<camera_context::CameraCalibration>& cam_info_vector,
        const feature_tracks::CurrentFrameObservation& dc_obs,
        const std::vector<bool>& valid_camera,
        const double max_reprojection_error_px
    )
    {
        const std::size_t num_cams = dc_obs.m_camera_observations.size();
        bool has_observation = false;

        for (std::size_t cam_local_idx {0}; cam_local_idx < num_cams; ++cam_local_idx)
        {
            if (!valid_camera[cam_local_idx])
            {
                continue;
            }

            if (pt_id >= dc_obs.m_camera_observations[cam_local_idx].m_keypoints.size())
            {
                continue;
            }

            const std::size_t cam_id = dc_obs.m_camera_observations[cam_local_idx].m_camera_id;
            if (cam_id >= cam_info_vector.size())
            {
                continue;
            }

            has_observation = true;

            const auto& cam_calib = cam_info_vector[cam_id];
            const auto& kp_obs = dc_obs.m_camera_observations[cam_local_idx].m_keypoints[pt_id];

            const Eigen::Vector3d X_cam =
                cam_calib.m_Rot.transpose() * (X_i - cam_calib.m_C);

            if (X_cam.z() <= 0.0)
            {
                return false;
            }

            const Eigen::Vector3d x_proj_h = cam_calib.m_K * X_cam;
            const Eigen::Vector2d x_proj {
                x_proj_h.x() / x_proj_h.z(),
                x_proj_h.y() / x_proj_h.z()
            };
            const Eigen::Vector2d x_obs {kp_obs.x, kp_obs.y};
            const double reprojection_error = (x_obs - x_proj).norm();

            if (reprojection_error > max_reprojection_error_px)
            {
                return false;
            }
        }

        return has_observation;
    }

    bot_interfaces::msg::PointWithCovarianceArray triangulationLOSTU(
        const std::size_t ref_local_idx,
        const std::vector<camera_context::CameraCalibration>& cam_info_vector,
        const feature_tracks::CurrentFrameObservation& dc_obs
    )
    {
        bot_interfaces::msg::PointWithCovarianceArray triangulated_points {};

        if (dc_obs.m_camera_observations.size() < 2 || ref_local_idx >= dc_obs.m_camera_observations.size())
        {
            return triangulated_points;
        }

        const std::size_t fallback_pair_local_idx = (ref_local_idx == 0) ? 1 : 0;
        const std::size_t num_cams = dc_obs.m_camera_observations.size();
        const std::size_t num_points = dc_obs.m_camera_observations[ref_local_idx].m_keypoints.size();

        triangulated_points.x.reserve(num_points);
        triangulated_points.y.reserve(num_points);
        triangulated_points.z.reserve(num_points);
        triangulated_points.covariance_xx.reserve(num_points);
        triangulated_points.covariance_xy.reserve(num_points);
        triangulated_points.covariance_xz.reserve(num_points);
        triangulated_points.covariance_yy.reserve(num_points);
        triangulated_points.covariance_yz.reserve(num_points);
        triangulated_points.covariance_zz.reserve(num_points);

        std::vector<std::size_t> pair_local_indices(num_cams, ref_local_idx);
        std::vector<bool> valid_camera(num_cams, false);
        std::vector<CasADi::OuterLoopPrecompute> outer_cache(num_cams);

        for (std::size_t cam_local_idx {0}; cam_local_idx < num_cams; ++cam_local_idx)
        {
            const std::size_t pair_local_idx = (cam_local_idx != ref_local_idx)
                ? ref_local_idx
                : fallback_pair_local_idx;
            pair_local_indices[cam_local_idx] = pair_local_idx;

            const std::size_t cam_id = dc_obs.m_camera_observations[cam_local_idx].m_camera_id;
            const std::size_t pair_cam_id = dc_obs.m_camera_observations[pair_local_idx].m_camera_id;

            if (cam_id >= cam_info_vector.size() || pair_cam_id >= cam_info_vector.size())
            {
                continue;
            }

            const auto& cam_calib = cam_info_vector[cam_id];
            const auto& pair_cam_calib = cam_info_vector[pair_cam_id];

            const CasADi::CameraPose cam_pose {
                cam_calib.m_Rot.transpose(),
                cam_calib.m_C
            };

            const CasADi::CameraPose pair_cam_pose {
                pair_cam_calib.m_Rot.transpose(),
                pair_cam_calib.m_C
            };

            outer_cache[cam_local_idx] = CasADi::runOuterLoopPrecompute(
                cam_calib,
                pair_cam_calib,
                cam_pose,
                pair_cam_pose,
                cam_calib.m_Sigma_C,
                cam_calib.m_Sigma_theta,
                cam_calib.m_Sigma_kappa
            );
            valid_camera[cam_local_idx] = true;
        }

        for (std::size_t pt_id {0}; pt_id < num_points; ++pt_id)
        {
            Eigen::Matrix3d sum_lhs = Eigen::Matrix3d::Zero();
            Eigen::Vector3d sum_rhs = Eigen::Vector3d::Zero();
            bool has_measurement = false;

            for (std::size_t cam_local_idx {0}; cam_local_idx < num_cams; ++cam_local_idx)
            {
                if (!valid_camera[cam_local_idx])
                {
                    continue;
                }

                const std::size_t pair_local_idx = pair_local_indices[cam_local_idx];
                const std::size_t cam_id = dc_obs.m_camera_observations[cam_local_idx].m_camera_id;
                if (
                    pt_id >= dc_obs.m_camera_observations[cam_local_idx].m_keypoints.size() ||
                    pt_id >= dc_obs.m_camera_observations[pair_local_idx].m_keypoints.size()
                )
                {
                    continue;
                }

                const auto& kp = dc_obs.m_camera_observations[cam_local_idx].m_keypoints[pt_id];
                const auto& kp_pair = dc_obs.m_camera_observations[pair_local_idx].m_keypoints[pt_id];

                const CasADi::InnerLoopCompute inner_loop_out = CasADi::runInnerLoopCompute(
                    outer_cache[cam_local_idx],
                    kp,
                    kp_pair
                );

                const auto sigma_res_cod = inner_loop_out.Sigma_res.completeOrthogonalDecomposition();
                const Eigen::Matrix3d sigma_res_pinv = sigma_res_cod.pseudoInverse();

                const Eigen::Vector3d x {kp.x, kp.y, 1.0};
                const Eigen::Vector3d u = outer_cache[cam_local_idx].Kinv * x;
                const Eigen::Matrix3d skew_u = (Eigen::Matrix3d {}
                    << 0.0,    -u.z(),  u.y(),
                       u.z(),   0.0,   -u.x(),
                      -u.y(),   u.x(),  0.0
                ).finished();

                const Eigen::Matrix3d lhs_term =
                    outer_cache[cam_local_idx].R.transpose() *
                    skew_u *
                    sigma_res_pinv *
                    skew_u *
                    outer_cache[cam_local_idx].R;

                const Eigen::Vector3d rhs_term = lhs_term * cam_info_vector[cam_id].m_C;

                sum_lhs += lhs_term;
                sum_rhs += rhs_term;
                has_measurement = true;
            }

            if (!has_measurement)
            {
                continue;
            }

            const auto lhs_cod = sum_lhs.completeOrthogonalDecomposition();
            const Eigen::Vector3d X_i = lhs_cod.solve(sum_rhs);
            if(!cheiralityCheck(X_i))
            {
                continue;
            }
            if(!reprojectionErrorCheck(X_i, pt_id, cam_info_vector, dc_obs, valid_camera))
            {
                continue;
            }

            const Eigen::Matrix3d lhs_pinv = lhs_cod.pseudoInverse();
            const Eigen::Matrix3d Sigma_X_i = -lhs_pinv;
            if(!finitenessCheck(X_i, Sigma_X_i) || !symPosSemiDefCheck(Sigma_X_i))
            {
                continue;
            }

            triangulated_points.x.push_back(static_cast<float>(X_i.x()));
            triangulated_points.y.push_back(static_cast<float>(X_i.y()));
            triangulated_points.z.push_back(static_cast<float>(X_i.z()));
            triangulated_points.covariance_xx.push_back(static_cast<float>(Sigma_X_i(0, 0)));
            triangulated_points.covariance_xy.push_back(static_cast<float>(0.5 * (Sigma_X_i(0, 1) + Sigma_X_i(1, 0))));
            triangulated_points.covariance_xz.push_back(static_cast<float>(0.5 * (Sigma_X_i(0, 2) + Sigma_X_i(2, 0))));
            triangulated_points.covariance_yy.push_back(static_cast<float>(Sigma_X_i(1, 1)));
            triangulated_points.covariance_yz.push_back(static_cast<float>(0.5 * (Sigma_X_i(1, 2) + Sigma_X_i(2, 1))));
            triangulated_points.covariance_zz.push_back(static_cast<float>(Sigma_X_i(2, 2)));
        }

        return triangulated_points;
    }
}
