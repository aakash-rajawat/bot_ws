#ifndef TRIANGULATION_HPP
#define TRIANGULATION_HPP

#include "bot_vision/camera_context.hpp"
#include "bot_vision/feature_tracks.hpp"

#include <bot_interfaces/msg/point_with_covariance_array.hpp>
#include <Eigen/Dense>


namespace bot_vision::triangulation
{
    bool finitenessCheck(const Eigen::Vector3d& X_i, const Eigen::Matrix3d& Sigma_X_i);
    bool cheiralityCheck(const Eigen::Vector3d& X_i);
    bool symPosSemiDefCheck(
        const Eigen::Matrix3d& Sigma_X_i, 
        const double symmetric_tolerance=1e-9, 
        const double positive_eigenvalue_tolerance=1e-12
    );
    bool reprojectionErrorCheck(
        const Eigen::Vector3d& X_i,
        const std::size_t pt_id,
        const std::vector<camera_context::CameraCalibration>& cam_info_vector,
        const feature_tracks::CurrentFrameObservation& dc_obs,
        const std::vector<bool>& valid_camera,
        const double max_reprojection_error_px=2.0
    );

    bot_interfaces::msg::PointWithCovarianceArray triangulationLOSTU(
        const std::size_t ref_local_idx,
        const std::vector<camera_context::CameraCalibration>& cam_info_vector,
        const feature_tracks::CurrentFrameObservation& dc_obs
    );
}


#endif
