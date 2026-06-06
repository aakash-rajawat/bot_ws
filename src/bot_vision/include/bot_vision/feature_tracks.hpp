#ifndef FEATURE_TRACKS_HPP
#define FEATURE_TRACKS_HPP

#include <vector>
#include <cstddef>
#include <opencv2/features2d.hpp>
#include <Eigen/Dense>
#include <bot_interfaces/msg/matched_view_pair.hpp>

namespace bot_vision::feature_tracks
{
    struct Keypoint
    {
        double x {};
        double y {};
        Eigen::Matrix2d covariance {Eigen::Matrix2d::Zero()};
    };

    struct CameraObservation
    {
        std::size_t m_camera_id {};
        std::vector<Keypoint> m_keypoints {};
    };

    struct CurrentFrameObservation
    {
        double m_time {};
        std::vector<CameraObservation> m_camera_observations {};

        void build(
            double time,
            std::size_t ref_local_idx,
            const std::vector<std::size_t>& selected_cam_ids,
            const std::vector<std::vector<cv::KeyPoint>>& keypoints,
            const std::vector<std::vector<cv::DMatch>>& filtered_matches,
            double sigma_x,
            double sigma_y
        );

        void build(
            double time,
            std::size_t ref_cam_id,
            const std::vector<std::size_t>& selected_cam_ids,
            const std::vector<bot_interfaces::msg::MatchedViewPair>& matched_pairs,
            double sigma_x,
            double sigma_y
        );
    };
}


#endif
