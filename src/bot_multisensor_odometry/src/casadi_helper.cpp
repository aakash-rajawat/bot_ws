#include "bot_multisensor_odometry/casadi_helper.hpp"

#include "bot_multisensor_odometry/ua_lidar_point_cloud.hpp"
#include "lidar_point_cloud_staged.h"

#include <array>
#include <cmath>
#include <stdexcept>

namespace bot_multisensor_odometry::CasADi
{
    Eigen::Matrix3d makeSigmaC(const LidarModel& lidar)
    {
        Eigen::Matrix3d Sigma_c = Eigen::Matrix3d::Zero();
        Sigma_c(0, 0) = lidar.m_sigma_c_x * lidar.m_sigma_c_x;
        Sigma_c(1, 1) = lidar.m_sigma_c_y * lidar.m_sigma_c_y;
        Sigma_c(2, 2) = lidar.m_sigma_c_z * lidar.m_sigma_c_z;
        return Sigma_c;
    }

    Eigen::Matrix3d makeSigmaPhi(const LidarModel& lidar)
    {
        Eigen::Matrix3d Sigma_phi = Eigen::Matrix3d::Zero();
        Sigma_phi(0, 0) = lidar.m_sigma_phi_x * lidar.m_sigma_phi_x;
        Sigma_phi(1, 1) = lidar.m_sigma_phi_y * lidar.m_sigma_phi_y;
        Sigma_phi(2, 2) = lidar.m_sigma_phi_z * lidar.m_sigma_phi_z;
        return Sigma_phi;
    }

    bool isValidRange(const LidarModel& lidar, float rho_i)
    {
        return std::isfinite(rho_i)
            && rho_i >= lidar.m_range_min
            && rho_i <= lidar.m_range_max;
    }

    PerLidarPrecompute runPerLidarPrecompute(const LidarModel& lidar)
    {
        PerLidarPrecompute out {};

        const Eigen::Matrix3d Sigma_c = makeSigmaC(lidar);
        const Eigen::Matrix3d Sigma_phi = makeSigmaPhi(lidar);

        std::array<const casadi_real*, per_lidar_precompute_SZ_ARG> arg {};
        std::array<casadi_real*, per_lidar_precompute_SZ_RES> res {};
        std::array<casadi_int, per_lidar_precompute_SZ_IW> iw {};
        std::array<casadi_real, per_lidar_precompute_SZ_W> w {};

        arg[0] = &lidar.m_theta_min;
        arg[1] = &lidar.m_Delta_theta;
        arg[2] = &lidar.m_b_rho;
        arg[3] = &lidar.m_s_rho;
        arg[4] = &lidar.m_b_theta;
        arg[5] = lidar.m_lidar_center.data();
        arg[6] = lidar.m_Rot.data();
        arg[7] = Sigma_c.data();
        arg[8] = Sigma_phi.data();
        arg[9] = &lidar.m_sigma_b_rho;
        arg[10] = &lidar.m_sigma_s_rho;
        arg[11] = &lidar.m_sigma_b_theta;

        res[0] = &out.theta_min;
        res[1] = &out.delta_theta;
        res[2] = &out.b_rho;
        res[3] = &out.s_rho;
        res[4] = &out.b_theta;
        res[5] = out.c.data();
        res[6] = out.Rt.data();
        res[7] = out.Sigma_c.data();
        res[8] = out.Sigma_phi.data();
        res[9] = &out.sigma_b_rho_sq;
        res[10] = &out.sigma_s_rho_sq;
        res[11] = &out.sigma_b_theta_sq;

        const int mem = per_lidar_precompute_checkout();
        if (mem < 0)
        {
            throw std::runtime_error("per_lidar_precompute_checkout failed");
        }

        const int status = per_lidar_precompute(
            arg.data(),
            res.data(),
            iw.data(),
            w.data(),
            mem
        );

        per_lidar_precompute_release(mem);

        if (status != 0)
        {
            throw std::runtime_error("per_lidar_precompute failed");
        }

        return out;
    }

    PerRangeCompute runPerRangeCompute(
        const PerLidarPrecompute& precompute,
        std::size_t beam_index,
        float rho_i,
        double sigma_rho_i
    )
    {
        PerRangeCompute out {};

        const casadi_real beam_index_real = static_cast<casadi_real>(beam_index);
        const casadi_real rho_i_real = static_cast<casadi_real>(rho_i);
        const casadi_real sigma_rho_i_real = static_cast<casadi_real>(sigma_rho_i);

        std::array<const casadi_real*, per_range_compute_SZ_ARG> arg {};
        std::array<casadi_real*, per_range_compute_SZ_RES> res {};
        std::array<casadi_int, per_range_compute_SZ_IW> iw {};
        std::array<casadi_real, per_range_compute_SZ_W> w {};

        arg[0] = &precompute.theta_min;
        arg[1] = &precompute.delta_theta;
        arg[2] = &precompute.b_rho;
        arg[3] = &precompute.s_rho;
        arg[4] = &precompute.b_theta;
        arg[5] = precompute.c.data();
        arg[6] = precompute.Rt.data();
        arg[7] = precompute.Sigma_c.data();
        arg[8] = precompute.Sigma_phi.data();
        arg[9] = &precompute.sigma_b_rho_sq;
        arg[10] = &precompute.sigma_s_rho_sq;
        arg[11] = &precompute.sigma_b_theta_sq;
        arg[12] = &beam_index_real;
        arg[13] = &rho_i_real;
        arg[14] = &sigma_rho_i_real;

        res[0] = out.X_i.data();
        res[1] = out.Sigma_X_i.data();

        const int mem = per_range_compute_checkout();
        if (mem < 0)
        {
            throw std::runtime_error("per_range_compute_checkout failed");
        }

        const int status = per_range_compute(
            arg.data(),
            res.data(),
            iw.data(),
            w.data(),
            mem
        );

        per_range_compute_release(mem);

        if (status != 0)
        {
            throw std::runtime_error("per_range_compute failed");
        }

        return out;
    }

    bot_interfaces::msg::PointWithCovariance beamToPointWithCovariance(
        const PerRangeCompute& beam
    )
    {
        bot_interfaces::msg::PointWithCovariance point {};

        point.x = static_cast<float>(beam.X_i.x());
        point.y = static_cast<float>(beam.X_i.y());
        point.z = static_cast<float>(beam.X_i.z());

        point.covariance_xx = static_cast<float>(beam.Sigma_X_i(0, 0));
        point.covariance_xy = static_cast<float>(0.5 * (beam.Sigma_X_i(0, 1) + beam.Sigma_X_i(1, 0)));
        point.covariance_xz = static_cast<float>(0.5 * (beam.Sigma_X_i(0, 2) + beam.Sigma_X_i(2, 0)));
        point.covariance_yy = static_cast<float>(beam.Sigma_X_i(1, 1));
        point.covariance_yz = static_cast<float>(0.5 * (beam.Sigma_X_i(1, 2) + beam.Sigma_X_i(2, 1)));
        point.covariance_zz = static_cast<float>(beam.Sigma_X_i(2, 2));

        return point;
    }
}
