#include "bot_vision/feature_tracks.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace bot_vision::feature_tracks
{
    namespace
    {
        struct PixelKey
        {
            std::int64_t x {};
            std::int64_t y {};

            bool operator==(const PixelKey& other) const
            {
                return x == other.x && y == other.y;
            }
        };

        struct PixelKeyHash
        {
            std::size_t operator()(const PixelKey& key) const
            {
                const auto hx = std::hash<std::int64_t> {}(key.x);
                const auto hy = std::hash<std::int64_t> {}(key.y);
                return hx ^ (hy << 1);
            }
        };

        PixelKey makePixelKey(const bot_interfaces::msg::Point2d& pt)
        {
            return PixelKey {
                static_cast<std::int64_t>(std::llround(static_cast<double>(pt.x) * 1000.0)),
                static_cast<std::int64_t>(std::llround(static_cast<double>(pt.y) * 1000.0))
            };
        }

        bool pairContainsRefCam(
            const bot_interfaces::msg::MatchedViewPair& pair,
            std::size_t ref_cam_id
        )
        {
            return pair.cam_id_a == ref_cam_id || pair.cam_id_b == ref_cam_id;
        }

        std::size_t otherCamId(
            const bot_interfaces::msg::MatchedViewPair& pair,
            std::size_t ref_cam_id
        )
        {
            return (pair.cam_id_a == ref_cam_id) ? pair.cam_id_b : pair.cam_id_a;
        }

        const std::vector<bot_interfaces::msg::Point2d>& refPoints(
            const bot_interfaces::msg::MatchedViewPair& pair,
            std::size_t ref_cam_id
        )
        {
            return (pair.cam_id_a == ref_cam_id) ? pair.points_a : pair.points_b;
        }

        const std::vector<bot_interfaces::msg::Point2d>& otherPoints(
            const bot_interfaces::msg::MatchedViewPair& pair,
            std::size_t ref_cam_id
        )
        {
            return (pair.cam_id_a == ref_cam_id) ? pair.points_b : pair.points_a;
        }
    }


    void CurrentFrameObservation::build(
        double time,
        std::size_t ref_local_idx,
        const std::vector<std::size_t>& selected_cam_ids,
        const std::vector<std::vector<cv::KeyPoint>>& keypoints,
        const std::vector<std::vector<cv::DMatch>>& filtered_matches,
        double sigma_x,
        double sigma_y
    )
    {
        this->m_time = time;
        this->m_camera_observations.resize(selected_cam_ids.size());

        for (std::size_t local_idx = 0; local_idx < selected_cam_ids.size(); ++local_idx)
        {
            this->m_camera_observations[local_idx].m_camera_id = selected_cam_ids[local_idx];
        }

        if (selected_cam_ids.size() < 2)
        {
            return;
        }

        Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
        covariance(0, 0) = sigma_x * sigma_x;
        covariance(1, 1) = sigma_y * sigma_y;



        std::vector<std::unordered_map<int, int>> match_lookup(selected_cam_ids.size());

        std::size_t anchor_local_idx = selected_cam_ids.size();

        for (std::size_t local_idx = 0; local_idx < selected_cam_ids.size(); ++local_idx)
        {
            if (local_idx == ref_local_idx)
            {
                continue;
            }

            if (filtered_matches[local_idx].empty())
            {
                return;
            }

            if (anchor_local_idx == selected_cam_ids.size())
            {
                anchor_local_idx = local_idx;
            }

            for (const auto& match : filtered_matches[local_idx])
            {
                match_lookup[local_idx][match.queryIdx] = match.trainIdx;
            }
        }

        if (anchor_local_idx == selected_cam_ids.size())
        {
            return;
        }

        for (const auto& [query_idx, _] : match_lookup[anchor_local_idx])
        {
            bool seen_in_all = true;

            for (std::size_t local_idx = 0; local_idx < selected_cam_ids.size(); ++local_idx)
            {
                if (local_idx == ref_local_idx)
                {
                    continue;
                }

                if (match_lookup[local_idx].find(query_idx) == match_lookup[local_idx].end())
                {
                    seen_in_all = false;
                    break;
                }
            }

            if (!seen_in_all)
            {
                continue;
            }

            const auto& ref_kp = keypoints[ref_local_idx][query_idx];
            this->m_camera_observations[ref_local_idx].m_keypoints.push_back(
                Keypoint{ref_kp.pt.x, ref_kp.pt.y, covariance}
            );

            for (std::size_t local_idx = 0; local_idx < selected_cam_ids.size(); ++local_idx)
            {
                if (local_idx == ref_local_idx)
                {
                    continue;
                }

                const int train_idx = match_lookup[local_idx].at(query_idx);
                const auto& kp = keypoints[local_idx][train_idx];

                this->m_camera_observations[local_idx].m_keypoints.push_back(
                    Keypoint{kp.pt.x, kp.pt.y, covariance}
                );
            }
        }

        return;
    }

    void CurrentFrameObservation::build(
        double time,
        std::size_t ref_cam_id,
        const std::vector<std::size_t>& selected_cam_ids,
        const std::vector<bot_interfaces::msg::MatchedViewPair>& matched_pairs,
        double sigma_x,
        double sigma_y
    )
    {
        this->m_time = time;
        this->m_camera_observations.clear();
        this->m_camera_observations.resize(selected_cam_ids.size());

        for(std::size_t local_idx = 0; local_idx < selected_cam_ids.size(); ++local_idx)
        {
            this->m_camera_observations[local_idx].m_camera_id = selected_cam_ids[local_idx];
        }

        if(selected_cam_ids.size() < 2)
        {
            return;
        }

        const auto ref_it = std::find(
            selected_cam_ids.begin(),
            selected_cam_ids.end(),
            ref_cam_id
        );

        if(ref_it == selected_cam_ids.end())
        {
            return;
        }

        const std::size_t ref_local_idx = std::distance(selected_cam_ids.begin(), ref_it);

        Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
        covariance(0, 0) = sigma_x * sigma_x;
        covariance(1, 1) = sigma_y * sigma_y;

        std::vector<const bot_interfaces::msg::MatchedViewPair*> pair_by_local_idx(
            selected_cam_ids.size(),
            nullptr
        );

        for(std::size_t local_idx = 0; local_idx < selected_cam_ids.size(); ++local_idx)
        {
            if(local_idx == ref_local_idx)
            {
                continue;
            }

            const std::size_t other_cam_id = selected_cam_ids[local_idx];

            const auto pair_it = std::find_if(
                matched_pairs.begin(),
                matched_pairs.end(),
                [ref_cam_id, other_cam_id](const auto& pair)
                {
                    return pairContainsRefCam(pair, ref_cam_id) &&
                        otherCamId(pair, ref_cam_id) == other_cam_id;
                }
            );

            if(pair_it == matched_pairs.end())
            {
                return;
            }

            const auto& ref_pts = refPoints(*pair_it, ref_cam_id);
            const auto& other_pts = otherPoints(*pair_it, ref_cam_id);

            if(ref_pts.empty() || ref_pts.size() != other_pts.size())
            {
                return;
            }

            pair_by_local_idx[local_idx] = &(*pair_it);
        }

        std::size_t anchor_local_idx = selected_cam_ids.size();
        for(std::size_t local_idx = 0; local_idx < selected_cam_ids.size(); ++local_idx)
        {
            if(local_idx == ref_local_idx)
            {
                continue;
            }

            if(pair_by_local_idx[local_idx] != nullptr)
            {
                anchor_local_idx = local_idx;
                break;
            }
        }

        if(anchor_local_idx == selected_cam_ids.size())
        {
            return;
        }

        using RefPointLookup = std::unordered_map<PixelKey, std::size_t, PixelKeyHash>;
        std::vector<RefPointLookup> ref_lookup(selected_cam_ids.size());

        for(std::size_t local_idx = 0; local_idx < selected_cam_ids.size(); ++local_idx)
        {
            if(local_idx == ref_local_idx || pair_by_local_idx[local_idx] == nullptr)
            {
                continue;
            }

            const auto& ref_pts = refPoints(*pair_by_local_idx[local_idx], ref_cam_id);
            auto& lookup = ref_lookup[local_idx];

            for(std::size_t pt_idx = 0; pt_idx < ref_pts.size(); ++pt_idx)
            {
                lookup[makePixelKey(ref_pts[pt_idx])] = pt_idx;
            }
        }

        const auto& anchor_ref_pts = refPoints(*pair_by_local_idx[anchor_local_idx], ref_cam_id);

        for(std::size_t anchor_pt_idx = 0; anchor_pt_idx < anchor_ref_pts.size(); ++anchor_pt_idx)
        {
            const PixelKey ref_key = makePixelKey(anchor_ref_pts[anchor_pt_idx]);
            bool seen_in_all = true;
            std::vector<std::size_t> matched_pt_idx(selected_cam_ids.size(), 0);
            matched_pt_idx[anchor_local_idx] = anchor_pt_idx;

            for(std::size_t local_idx = 0; local_idx < selected_cam_ids.size(); ++local_idx)
            {
                if(local_idx == ref_local_idx)
                {
                    continue;
                }

                const auto lookup_it = ref_lookup[local_idx].find(ref_key);
                if(lookup_it == ref_lookup[local_idx].end())
                {
                    seen_in_all = false;
                    break;
                }

                matched_pt_idx[local_idx] = lookup_it->second;
            }

            if(!seen_in_all)
            {
                continue;
            }

            const auto& ref_pt = anchor_ref_pts[anchor_pt_idx];
            this->m_camera_observations[ref_local_idx].m_keypoints.push_back(
                Keypoint {
                    static_cast<double>(ref_pt.x),
                    static_cast<double>(ref_pt.y),
                    covariance
                }
            );

            for(std::size_t local_idx = 0; local_idx < selected_cam_ids.size(); ++local_idx)
            {
                if(local_idx == ref_local_idx)
                {
                    continue;
                }

                const auto& other_pts = otherPoints(*pair_by_local_idx[local_idx], ref_cam_id);
                const auto& pt = other_pts[matched_pt_idx[local_idx]];

                this->m_camera_observations[local_idx].m_keypoints.push_back(
                    Keypoint {
                        static_cast<double>(pt.x),
                        static_cast<double>(pt.y),
                        covariance
                    }
                );
            }
        }

        return;
    }
}
