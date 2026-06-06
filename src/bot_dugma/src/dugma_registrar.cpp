#include "bot_dugma/dugma_registrar.hpp"
#include "bot_dugma/normalize.hpp"
#include "bot_dugma/initialize.hpp"
#include "bot_dugma/e_step.hpp"
#include "bot_dugma/m_step.hpp"
#include "bot_dugma/uncertainty_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace bot_dugma
{
    namespace
    {
        PoseEstimate composeDeltaPose(
            const PoseEstimate& current_pose,
            const PoseEstimate& delta_pose
        )
        {
            PoseEstimate out = delta_pose;

            for(int r = 0; r < 3; ++r)
            {
                for(int c = 0; c < 3; ++c)
                {
                    out.m_R[3 * r + c] =
                        delta_pose.m_R[3 * r + 0] * current_pose.m_R[0 * 3 + c] +
                        delta_pose.m_R[3 * r + 1] * current_pose.m_R[1 * 3 + c] +
                        delta_pose.m_R[3 * r + 2] * current_pose.m_R[2 * 3 + c];
                }
            }

            for(int r = 0; r < 3; ++r)
            {
                out.m_t[r] =
                    delta_pose.m_R[3 * r + 0] * current_pose.m_t[0] +
                    delta_pose.m_R[3 * r + 1] * current_pose.m_t[1] +
                    delta_pose.m_R[3 * r + 2] * current_pose.m_t[2] +
                    delta_pose.m_t[r];
            }

            return out;
        }

        bool outerConverged(
            const PoseEstimate& old_pose,
            const PoseEstimate& new_pose,
            float translation_tol,
            float rotation_tol_rad
        )
        {
            const float dx = new_pose.m_t[0] - old_pose.m_t[0];
            const float dy = new_pose.m_t[1] - old_pose.m_t[1];
            const float dz = new_pose.m_t[2] - old_pose.m_t[2];

            const float trans_delta = std::sqrt(dx * dx + dy * dy + dz * dz);

            const float r00 =
                old_pose.m_R[0] * new_pose.m_R[0] +
                old_pose.m_R[3] * new_pose.m_R[3] +
                old_pose.m_R[6] * new_pose.m_R[6];

            const float r11 =
                old_pose.m_R[1] * new_pose.m_R[1] +
                old_pose.m_R[4] * new_pose.m_R[4] +
                old_pose.m_R[7] * new_pose.m_R[7];

            const float r22 =
                old_pose.m_R[2] * new_pose.m_R[2] +
                old_pose.m_R[5] * new_pose.m_R[5] +
                old_pose.m_R[8] * new_pose.m_R[8];

            const float trace = r00 + r11 + r22;
            const float cos_angle = std::clamp(
                (trace - 1.0f) * 0.5f,
                -1.0f,
                1.0f
            );
            const float rot_delta = std::acos(cos_angle);

            return trans_delta < translation_tol &&
                   rot_delta < rotation_tol_rad;
        }
    }

    DugmaRegistrar::DugmaRegistrar(const DugmaRegistrarConfig& config)
    {
        // stuff for this constructor pbject
        m_config = config;
    }

    PoseEstimate DugmaRegistrar::estimate(
        const PointCloudView& in_X,
        const PointCloudView& in_Y,
        const PoseEstimate* init_guess
    )
    {
        // normalize inputs
        NormalizedPointClouds norm_pt_clouds {};
        norm_pt_clouds.normalizePointClouds(in_X, in_Y);

        // load point clouds to device, 
        // then precompute these terms: 
        // inv_Sigma_Y0, sqrt(1/det_Y0), inv_Sigma_X, sqrt(1/det_X) V x_n, y_m
        DeviceRegistrationData device_reg_data(
            norm_pt_clouds.m_X, 
            norm_pt_clouds.m_Y
        );

        // Repeat until convergence --------------------------------
        PoseEstimate pose {};
        if(init_guess != nullptr)
        {
            pose = normalizePose(*init_guess, norm_pt_clouds.m_info);
        }

        pose.m_is_converged = false;
        pose.m_iter = 0;

        for(std::size_t it {0}; it < m_config.m_max_iter; ++it)
        {
            const PoseEstimate old_pose = pose;

            // E-STEP
            EStep e_step(device_reg_data, pose);

            // M-STEP
            MStep m_step;
            PoseEstimate delta_init {};
            PoseEstimate delta_pose = m_step.optimize(
                e_step.basisCoeffs(),
                delta_init
            );

            PoseEstimate new_pose = composeDeltaPose(pose, delta_pose);
            new_pose.m_is_converged = delta_pose.m_is_converged;
            new_pose.m_iter = delta_pose.m_iter;
            new_pose.m_energy = delta_pose.m_energy;

            if(!new_pose.m_is_converged)
            {
                pose = new_pose;
                break;
            }

            pose = new_pose;
            if(outerConverged(
                old_pose,
                pose,
                m_config.m_acc_tol_translatation,
                m_config.m_acc_tol_rotation
            ))
            {
                pose.m_is_converged = true;
                break;
            }

            pose.m_is_converged = false;
        }
        // ---------------------------------------------------------

        std::array<float, 21> pose_cov {};
        bool has_pose_cov = false;

        if(pose.m_is_converged)
        {
            EStep final_e_step(device_reg_data, pose);

            UncertaintyEstimator uncertainty_estimator(
                device_reg_data,
                pose,
                norm_pt_clouds.m_info
            );

            has_pose_cov = uncertainty_estimator.hasValidCovariance();
            if(has_pose_cov)
            {
                pose_cov = uncertainty_estimator.denormalizedPoseCovariance();
            }
        }

        PoseEstimate out = denormalizePose(pose, norm_pt_clouds.m_info);
        out.m_pose_cov = pose_cov;
        out.m_has_pose_cov = has_pose_cov;
        return out;
    }

    PoseEstimate DugmaRegistrar::estimate(
        const PointCloud& in_X,
        const PointCloud& in_Y,
        const PoseEstimate* init_guess
    )
    {
        return estimate(
            makePointCloudView(in_X),
            makePointCloudView(in_Y),
            init_guess
        );
    }
}
