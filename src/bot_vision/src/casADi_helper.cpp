#include "bot_vision/casADi_helper.hpp"

#include <array>
#include <stdexcept>


namespace bot_vision::CasADi
{
    OuterLoopPrecompute runOuterLoopPrecompute(
        const bot_vision::camera_context::CameraCalibration& cam_info,
        const bot_vision::camera_context::CameraCalibration& ref_cam_info,
        const CameraPose& cam_pose,
        const CameraPose& ref_cam_pose,
        const Eigen::Matrix3d& Sigma_C,
        const Eigen::Matrix3d& Sigma_theta,
        const Eigen::Matrix4d& Sigma_kappa
    )
    {
        OuterLoopPrecompute out {};

        const casadi_real fx = cam_info.m_K(0, 0);
        const casadi_real fy = cam_info.m_K(1, 1);
        const casadi_real cx = cam_info.m_K(0, 2);
        const casadi_real cy = cam_info.m_K(1, 2);

        const casadi_real fxp = ref_cam_info.m_K(0, 0);
        const casadi_real fyp = ref_cam_info.m_K(1, 1);
        const casadi_real cxp = ref_cam_info.m_K(0, 2);
        const casadi_real cyp = ref_cam_info.m_K(1, 2);

        std::array<const casadi_real*, triangulation_outer_precompute_SZ_ARG> arg {};
        std::array<casadi_real*, triangulation_outer_precompute_SZ_RES> res {};
        std::array<casadi_int, triangulation_outer_precompute_SZ_IW> iw {};
        std::array<casadi_real, triangulation_outer_precompute_SZ_W> w {};

        arg[0] = cam_pose.R.data();
        arg[1] = cam_pose.C.data();
        arg[2] = ref_cam_pose.C.data();
        arg[3] = &fx;
        arg[4] = &fy;
        arg[5] = &cx;
        arg[6] = &cy;
        arg[7] = &fxp;
        arg[8] = &fyp;
        arg[9] = &cxp;
        arg[10] = &cyp;
        arg[11] = Sigma_C.data();
        arg[12] = Sigma_theta.data();
        arg[13] = Sigma_kappa.data();

        res[0] = out.Kinv.data();
        res[1] = out.Kpinv.data();
        res[2] = out.R.data();
        res[3] = out.b.data();
        res[4] = out.dKinv_dfx.data();
        res[5] = out.dKinv_dfy.data();
        res[6] = out.dKinv_dcx.data();
        res[7] = out.dKinv_dcy.data();
        res[8] = out.Sigma_C.data();
        res[9] = out.Sigma_theta.data();
        res[10] = out.Sigma_kappa.data();

        const int mem = triangulation_outer_precompute_checkout();
        if (mem < 0)
        {
            throw std::runtime_error("triangulation_outer_precompute_checkout failed");
        }

        const int status = triangulation_outer_precompute(
            arg.data(),
            res.data(),
            iw.data(),
            w.data(),
            mem
        );

        triangulation_outer_precompute_release(mem);

        if (status != 0)
        {
            throw std::runtime_error("triangulation_outer_precompute failed");
        }

        return out;
    }

    InnerLoopCompute runInnerLoopCompute(
        const OuterLoopPrecompute& outer_precompute,
        const bot_vision::feature_tracks::Keypoint& keypoint,
        const bot_vision::feature_tracks::Keypoint& paired_keypoint
    )
    {
        InnerLoopCompute out {};

        const Eigen::Vector3d x {keypoint.x, keypoint.y, 1.0};
        const Eigen::Vector3d xp {paired_keypoint.x, paired_keypoint.y, 1.0};

        Eigen::Matrix3d Sigma_xdc = Eigen::Matrix3d::Zero();
        Sigma_xdc.topLeftCorner<2, 2>() = keypoint.covariance;

        std::array<const casadi_real*, triangulation_inner_sigma_res_SZ_ARG> arg {};
        std::array<casadi_real*, triangulation_inner_sigma_res_SZ_RES> res {};
        std::array<casadi_int, triangulation_inner_sigma_res_SZ_IW> iw {};
        std::array<casadi_real, triangulation_inner_sigma_res_SZ_W> w {};

        arg[0] = outer_precompute.Kinv.data();
        arg[1] = outer_precompute.Kpinv.data();
        arg[2] = outer_precompute.R.data();
        arg[3] = outer_precompute.b.data();
        arg[4] = outer_precompute.dKinv_dfx.data();
        arg[5] = outer_precompute.dKinv_dfy.data();
        arg[6] = outer_precompute.dKinv_dcx.data();
        arg[7] = outer_precompute.dKinv_dcy.data();
        arg[8] = outer_precompute.Sigma_C.data();
        arg[9] = outer_precompute.Sigma_theta.data();
        arg[10] = outer_precompute.Sigma_kappa.data();
        arg[11] = x.data();
        arg[12] = xp.data();
        arg[13] = Sigma_xdc.data();

        res[0] = out.a.data();
        res[1] = out.ap.data();
        res[2] = &out.rho;
        res[3] = out.J_xdc.data();
        res[4] = out.J_C.data();
        res[5] = out.J_theta.data();
        res[6] = out.J_kappa.data();
        res[7] = out.Sigma_res.data();

        const int mem = triangulation_inner_sigma_res_checkout();
        if (mem < 0)
        {
            throw std::runtime_error("triangulation_inner_sigma_res_checkout failed");
        }

        const int status = triangulation_inner_sigma_res(
            arg.data(),
            res.data(),
            iw.data(),
            w.data(),
            mem
        );

        triangulation_inner_sigma_res_release(mem);

        if (status != 0)
        {
            throw std::runtime_error("triangulation_inner_sigma_res failed");
        }

        return out;
    }
}
