#include "bot_vision/distortion_correction.hpp"
#include "bot_vision_generated/equidistant_undistorted_covariance.h"
#include "bot_vision_generated/plumb_bob_undistorted_covariance.h"
#include "bot_vision_generated/rational_polynomial_undistorted_covariance.h"


#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>

namespace bot_vision::distortion_correction
{
    feature_tracks::CurrentFrameObservation uncertaintyAwareDistortionCorrection(
        const std::vector<camera_context::CameraCalibration>& cam_info_vector,
        const feature_tracks::CurrentFrameObservation& observations
    )
    {
        feature_tracks::CurrentFrameObservation dc_obs {};
        dc_obs.m_camera_observations.resize(cam_info_vector.size());
        dc_obs.m_time = observations.m_time;

        for(std::size_t cam_idx {0}; cam_idx < cam_info_vector.size(); ++cam_idx)
        {
            dc_obs.m_camera_observations[cam_idx].m_camera_id = 
                observations.m_camera_observations[cam_idx].m_camera_id;

            dc_obs.m_camera_observations[cam_idx].m_keypoints.resize(
                observations.m_camera_observations[cam_idx].m_keypoints.size()
            );

            if(observations.m_camera_observations[cam_idx].m_keypoints.empty())
            {
                continue;
            }

            cv::Mat K = camera_context::toCvMat(cam_info_vector[cam_idx].m_K);
            double f_x = cam_info_vector[cam_idx].m_K(0, 0);
            double f_y = cam_info_vector[cam_idx].m_K(1, 1);
            double c_x = cam_info_vector[cam_idx].m_K(0, 2);
            double c_y = cam_info_vector[cam_idx].m_K(1, 2);

            Eigen::Matrix<double, 4, 1> intrinsics;
            intrinsics << f_x, f_y, c_x, c_y;

            double k_1 {0.0}, k_2 {0.0}, p_1 {0.0}, p_2 {0.0}, k_3 {0.0}, 
                k_4 {0.0}, k_5 {0.0}, k_6 {0.0};

            if(cam_info_vector[cam_idx].m_distortion_model == "equidistant")
            {
                k_1 = cam_info_vector[cam_idx].m_dist_params[0];
                k_2 = cam_info_vector[cam_idx].m_dist_params[1];
                k_3 = cam_info_vector[cam_idx].m_dist_params[2];
                k_4 = cam_info_vector[cam_idx].m_dist_params[3];

                Eigen::Matrix<double, 4, 1> distortion;
                distortion << k_1, k_2, k_3, k_4;

                std::vector<double> dstCoeffs {k_1, k_2, k_3, k_4};

                std::vector<cv::Point2f> input {};
                for(
                    std::size_t pt_idx {0}; 
                    pt_idx < observations.m_camera_observations[cam_idx].m_keypoints.size(); 
                    ++pt_idx
                )
                {
                    input.push_back(
                        cv::Point2f(
                            observations.m_camera_observations[cam_idx].m_keypoints[pt_idx].x,
                            observations.m_camera_observations[cam_idx].m_keypoints[pt_idx].y
                        )
                    );
                }

                std::vector<cv::Point2f> output(input.size());
                cv::fisheye::undistortPoints(input, output, K, dstCoeffs, cv::noArray(), K);

                for(
                    std::size_t pt_idx {0}; 
                    pt_idx < observations.m_camera_observations[cam_idx].m_keypoints.size(); 
                    ++pt_idx
                )
                {
                    Eigen::Matrix<double, 2, 1> uv;
                    uv << output[pt_idx].x, output[pt_idx].y;

                    dc_obs.m_camera_observations[cam_idx].m_keypoints[pt_idx].x = output[pt_idx].x;
                    dc_obs.m_camera_observations[cam_idx].m_keypoints[pt_idx].y = output[pt_idx].y;

                    Eigen::Matrix<double, 2, 2> sigma_d =
                        observations.m_camera_observations[cam_idx].m_keypoints[pt_idx].covariance;

                    bot_vision_generated::EquidistantUndistortedCovariance<double>(
                        uv,
                        intrinsics,
                        distortion,
                        sigma_d,
                        &dc_obs.m_camera_observations[cam_idx].m_keypoints[pt_idx].covariance
                    );
                }
            }   // end of equidistant model
            else if(cam_info_vector[cam_idx].m_distortion_model == "plumb_bob")
            {
                k_1 = cam_info_vector[cam_idx].m_dist_params[0];
                k_2 = cam_info_vector[cam_idx].m_dist_params[1];
                p_1 = cam_info_vector[cam_idx].m_dist_params[2];
                p_2 = cam_info_vector[cam_idx].m_dist_params[3];
                k_3 = cam_info_vector[cam_idx].m_dist_params[4];

                Eigen::Matrix<double, 5, 1> distortion;
                distortion << k_1, k_2, p_1, p_2, k_3;

                std::vector<double> dstCoeffs {k_1, k_2, p_1, p_2, k_3};

                std::vector<cv::Point2f> input {};
                for(
                    std::size_t pt_idx {0}; 
                    pt_idx < observations.m_camera_observations[cam_idx].m_keypoints.size(); 
                    ++pt_idx
                )
                {
                    input.push_back(
                        cv::Point2f(
                            observations.m_camera_observations[cam_idx].m_keypoints[pt_idx].x,
                            observations.m_camera_observations[cam_idx].m_keypoints[pt_idx].y
                        )
                    );
                }

                std::vector<cv::Point2f> output(input.size());
                cv::undistortPoints(input, output, K, dstCoeffs, cv::noArray(), K);

                for(
                    std::size_t pt_idx {0}; 
                    pt_idx < observations.m_camera_observations[cam_idx].m_keypoints.size(); 
                    ++pt_idx
                )
                {
                    Eigen::Matrix<double, 2, 1> uv;
                    uv << output[pt_idx].x, output[pt_idx].y;

                    dc_obs.m_camera_observations[cam_idx].m_keypoints[pt_idx].x = output[pt_idx].x;
                    dc_obs.m_camera_observations[cam_idx].m_keypoints[pt_idx].y = output[pt_idx].y;

                    Eigen::Matrix<double, 2, 2> sigma_d =
                        observations.m_camera_observations[cam_idx].m_keypoints[pt_idx].covariance;

                    bot_vision_generated::PlumbBobUndistortedCovariance<double>(
                        uv,
                        intrinsics,
                        distortion,
                        sigma_d,
                        &dc_obs.m_camera_observations[cam_idx].m_keypoints[pt_idx].covariance
                    );
                }
            }   // end of plum_bob model
            else if(cam_info_vector[cam_idx].m_distortion_model == "rational_polynomial")
            {
                k_1 = cam_info_vector[cam_idx].m_dist_params[0];
                k_2 = cam_info_vector[cam_idx].m_dist_params[1];
                p_1 = cam_info_vector[cam_idx].m_dist_params[2];
                p_2 = cam_info_vector[cam_idx].m_dist_params[3];
                k_3 = cam_info_vector[cam_idx].m_dist_params[4];
                k_4 = cam_info_vector[cam_idx].m_dist_params[5];
                k_5 = cam_info_vector[cam_idx].m_dist_params[6];
                k_6 = cam_info_vector[cam_idx].m_dist_params[7];

                Eigen::Matrix<double, 8, 1> distortion;
                distortion << k_1, k_2, p_1, p_2, k_3, k_4, k_5, k_6;

                std::vector<double> dstCoeffs {k_1, k_2, p_1, p_2, k_3, k_4, k_5, k_6};

                std::vector<cv::Point2f> input {};
                for(
                    std::size_t pt_idx {0}; 
                    pt_idx < observations.m_camera_observations[cam_idx].m_keypoints.size(); 
                    ++pt_idx
                )
                {
                    input.push_back(
                        cv::Point2f(
                            observations.m_camera_observations[cam_idx].m_keypoints[pt_idx].x,
                            observations.m_camera_observations[cam_idx].m_keypoints[pt_idx].y
                        )
                    );
                }

                std::vector<cv::Point2f> output(input.size());
                cv::undistortPoints(input, output, K, dstCoeffs, cv::noArray(), K);

                for(
                    std::size_t pt_idx {0}; 
                    pt_idx < observations.m_camera_observations[cam_idx].m_keypoints.size(); 
                    ++pt_idx
                )
                {
                    Eigen::Matrix<double, 2, 1> uv;
                    uv << output[pt_idx].x, output[pt_idx].y;

                    dc_obs.m_camera_observations[cam_idx].m_keypoints[pt_idx].x = output[pt_idx].x;
                    dc_obs.m_camera_observations[cam_idx].m_keypoints[pt_idx].y = output[pt_idx].y;

                    Eigen::Matrix<double, 2, 2> sigma_d =
                        observations.m_camera_observations[cam_idx].m_keypoints[pt_idx].covariance;

                    bot_vision_generated::RationalPolynomialUndistortedCovariance<double>(
                        uv,
                        intrinsics,
                        distortion,
                        sigma_d,
                        &dc_obs.m_camera_observations[cam_idx].m_keypoints[pt_idx].covariance
                    );
                }
            }   // end of rational_polynomial model
            else
            {
                // model different than the standard ROS models
                return dc_obs;
            }

        }

        return dc_obs;
    }
}
