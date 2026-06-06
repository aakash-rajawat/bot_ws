#include "bot_vision/correspondence.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>

namespace bot_vision::correspondence
{
    BFMatcherStereo::BFMatcherStereo(const MatchParams2D& params)
    {
        m_params = params;
        m_bfmatcher = cv::BFMatcher::create(m_params.norm_type, false);
    }

    std::vector<cv::DMatch> BFMatcherStereo::matcher(
        const std::vector<cv::KeyPoint>& keypoints1,
        const std::vector<cv::KeyPoint>& keypoints2,
        const cv::Mat& descriptors1,
        const cv::Mat& descriptors2
    )
    {
        std::vector<cv::DMatch> raw_matches {};
        if(
            descriptors1.empty() || descriptors2.empty()
            || descriptors1.rows <= 0 || descriptors2.rows <= 0
            || descriptors1.cols <= 0 || descriptors2.cols <= 0
            || descriptors1.type() != descriptors2.type()
        )
        {
            return raw_matches;
        }

        std::vector<std::vector<cv::DMatch>> knn_matches {};
        std::vector<cv::DMatch> reverse_matches {};
        try
        {
            m_bfmatcher->knnMatch(
                descriptors1,
                descriptors2,
                knn_matches,
                2
            );

            if(m_params.use_mutual_consistency)
            {
                m_bfmatcher->match(
                    descriptors2,
                    descriptors1,
                    reverse_matches
                );
            }
        }
        catch(const cv::Exception&)
        {
            return raw_matches;
        }

        std::vector<cv::DMatch> descriptor_filtered_matches {};
        descriptor_filtered_matches.reserve(knn_matches.size());

        for(const auto& match_pair : knn_matches)
        {
            if(match_pair.empty())
            {
                continue;
            }

            const auto& best = match_pair[0];

            if(m_params.use_ratio_test)
            {
                if(match_pair.size() < 2)
                {
                    continue;
                }

                const auto& second_best = match_pair[1];
                if(best.distance >= m_params.ratio_test_threshold * second_best.distance)
                {
                    continue;
                }
            }

            if(
                m_params.use_descriptor_distance_filter
                && best.distance > m_params.max_descriptor_distance
            )
            {
                continue;
            }

            if(m_params.use_mutual_consistency)
            {
                if(
                    best.trainIdx < 0
                    || static_cast<std::size_t>(best.trainIdx) >= reverse_matches.size()
                )
                {
                    continue;
                }

                if(reverse_matches[best.trainIdx].trainIdx != best.queryIdx)
                {
                    continue;
                }
            }

            descriptor_filtered_matches.push_back(best);
        }

        std::vector<cv::DMatch> filtered_matches {};
        filtered_matches.reserve(descriptor_filtered_matches.size());

        for(const auto& match : descriptor_filtered_matches)
        {
            const auto& keypoint1 = keypoints1[match.queryIdx];
            const auto& keypoint2 = keypoints2[match.trainIdx];

            const double row_diff = std::abs(keypoint1.pt.y - keypoint2.pt.y);
            const double disparity = keypoint1.pt.x - keypoint2.pt.x;

            if(m_params.use_epipolar_filter)
            {
                if(row_diff > m_params.max_row_diff)
                {
                    continue;
                }
                if(
                    disparity < m_params.min_disparity
                    || disparity > m_params.max_disparity
                )
                {
                    continue;
                }
            }
            filtered_matches.push_back(match);
        }

        if(m_params.use_fundamental_ransac && filtered_matches.size() >= 8)
        {
            try
            {
                std::vector<cv::Point2f> pts1 {};
                std::vector<cv::Point2f> pts2 {};
                pts1.reserve(filtered_matches.size());
                pts2.reserve(filtered_matches.size());

                for(const auto& match : filtered_matches)
                {
                    pts1.push_back(keypoints1[match.queryIdx].pt);
                    pts2.push_back(keypoints2[match.trainIdx].pt);
                }

                cv::Mat inlier_mask {};
                const cv::Mat F = cv::findFundamentalMat(
                    pts1,
                    pts2,
                    cv::FM_RANSAC,
                    m_params.ransac_reproj_threshold,
                    m_params.ransac_confidence,
                    inlier_mask
                );

                if(!F.empty() && !inlier_mask.empty())
                {
                    std::vector<cv::DMatch> ransac_filtered_matches {};
                    ransac_filtered_matches.reserve(filtered_matches.size());

                    for(int idx = 0; idx < static_cast<int>(inlier_mask.total()); ++idx)
                    {
                        if(inlier_mask.at<uchar>(idx) != 0)
                        {
                            ransac_filtered_matches.push_back(
                                filtered_matches[static_cast<std::size_t>(idx)]
                            );
                        }
                    }

                    filtered_matches = std::move(ransac_filtered_matches);
                }
            }
            catch(const cv::Exception&)
            {
                // Keep the pre-RANSAC matches for this frame.
            }
        }

        std::sort(
            filtered_matches.begin(),
            filtered_matches.end(),
            [](const cv::DMatch& lhs, const cv::DMatch& rhs)
            {
                return lhs.distance < rhs.distance;
            }
        );

        return filtered_matches;
    }
    
}   // namespace bot_vision::correspondence
