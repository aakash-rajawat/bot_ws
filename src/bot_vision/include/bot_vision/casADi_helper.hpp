#ifndef CASADI_HELPER_HPP
#define CASADI_HELPER_HPP

#include "triangulation_sigma_res_staged.h"
#include "bot_vision/camera_context.hpp"
#include "bot_vision/feature_tracks.hpp"

#include <Eigen/Dense>

namespace bot_vision::CasADi
{
    struct CameraPose
    {
        Eigen::Matrix3d R {Eigen::Matrix3d::Identity()};
        Eigen::Vector3d C {Eigen::Vector3d::Zero()};
    };

    struct OuterLoopPrecompute
    {
        Eigen::Matrix3d Kinv {Eigen::Matrix3d::Zero()};
        Eigen::Matrix3d Kpinv {Eigen::Matrix3d::Zero()};
        Eigen::Matrix3d R {Eigen::Matrix3d::Zero()};
        Eigen::Vector3d b {Eigen::Vector3d::Zero()};

        Eigen::Matrix3d dKinv_dfx {Eigen::Matrix3d::Zero()};
        Eigen::Matrix3d dKinv_dfy {Eigen::Matrix3d::Zero()};
        Eigen::Matrix3d dKinv_dcx {Eigen::Matrix3d::Zero()};
        Eigen::Matrix3d dKinv_dcy {Eigen::Matrix3d::Zero()};

        Eigen::Matrix3d Sigma_C {Eigen::Matrix3d::Zero()};
        Eigen::Matrix3d Sigma_theta {Eigen::Matrix3d::Zero()};
        Eigen::Matrix4d Sigma_kappa {Eigen::Matrix4d::Zero()};
    };

    struct InnerLoopCompute
    {
        Eigen::Vector3d a {Eigen::Vector3d::Zero()};
        Eigen::Vector3d ap {Eigen::Vector3d::Zero()};
        double rho {};

        Eigen::Matrix3d J_xdc {Eigen::Matrix3d::Zero()};
        Eigen::Matrix3d J_C {Eigen::Matrix3d::Zero()};
        Eigen::Matrix3d J_theta {Eigen::Matrix3d::Zero()};
        Eigen::Matrix<double, 3, 4> J_kappa {Eigen::Matrix<double, 3, 4>::Zero()};
        Eigen::Matrix3d Sigma_res {Eigen::Matrix3d::Zero()};
    };

    OuterLoopPrecompute runOuterLoopPrecompute(
        const bot_vision::camera_context::CameraCalibration& cam_info,
        const bot_vision::camera_context::CameraCalibration& ref_cam_info,
        const CameraPose& cam_pose,
        const CameraPose& ref_cam_pose,
        const Eigen::Matrix3d& Sigma_C,
        const Eigen::Matrix3d& Sigma_theta,
        const Eigen::Matrix4d& Sigma_kappa
    );

    InnerLoopCompute runInnerLoopCompute(
        const OuterLoopPrecompute& outer_precompute,
        const bot_vision::feature_tracks::Keypoint& keypoint,
        const bot_vision::feature_tracks::Keypoint& paired_keypoint
    );
}


#endif
