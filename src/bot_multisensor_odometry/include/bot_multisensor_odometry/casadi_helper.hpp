#ifndef BOT_MULTISENSOR_ODOMETRY_CASADI_HELPER_HPP
#define BOT_MULTISENSOR_ODOMETRY_CASADI_HELPER_HPP

#include <bot_interfaces/msg/point_with_covariance.hpp>

#include <Eigen/Dense>

#include <cstddef>

namespace bot_multisensor_odometry
{
    struct LidarModel;
}

namespace bot_multisensor_odometry::CasADi
{
    struct PerLidarPrecompute
    {
        double theta_min {};
        double delta_theta {};
        double b_rho {};
        double s_rho {};
        double b_theta {};

        Eigen::Vector3d c {Eigen::Vector3d::Zero()};
        Eigen::Matrix3d Rt {Eigen::Matrix3d::Zero()};
        Eigen::Matrix3d Sigma_c {Eigen::Matrix3d::Zero()};
        Eigen::Matrix3d Sigma_phi {Eigen::Matrix3d::Zero()};

        double sigma_b_rho_sq {};
        double sigma_s_rho_sq {};
        double sigma_b_theta_sq {};
    };

    struct PerRangeCompute
    {
        Eigen::Vector3d X_i {Eigen::Vector3d::Zero()};
        Eigen::Matrix3d Sigma_X_i {Eigen::Matrix3d::Zero()};
    };

    Eigen::Matrix3d makeSigmaC(const LidarModel& lidar);
    Eigen::Matrix3d makeSigmaPhi(const LidarModel& lidar);
    bool isValidRange(const LidarModel& lidar, float rho_i);

    PerLidarPrecompute runPerLidarPrecompute(const LidarModel& lidar);

    PerRangeCompute runPerRangeCompute(
        const PerLidarPrecompute& precompute,
        std::size_t beam_index,
        float rho_i,
        double sigma_rho_i
    );

    bot_interfaces::msg::PointWithCovariance beamToPointWithCovariance(
        const PerRangeCompute& beam
    );
}

#endif
