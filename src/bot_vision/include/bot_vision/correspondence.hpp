#ifndef CORRESPONDENCE_HPP
#define CORRESPONDENCE_HPP

#include <opencv2/features2d.hpp>
#include <opencv2/core/types.hpp>

namespace bot_vision::correspondence
{
    struct MatchParams2D
    {
        int norm_type {};
        bool use_ratio_test {true};
        double ratio_test_threshold {0.72};
        bool use_descriptor_distance_filter {true};
        double max_descriptor_distance {40.0};
        bool use_mutual_consistency {true};
        bool use_fundamental_ransac {true};
        double ransac_reproj_threshold {1.5};
        double ransac_confidence {0.995};
        bool use_epipolar_filter {false};
        double max_row_diff {0.0};
        double min_disparity {0.0};
        double max_disparity {0.0};
    };

    class BFMatcherStereo
    {
    public:
        explicit BFMatcherStereo(const MatchParams2D& params);
        std::vector<cv::DMatch> matcher(
            const std::vector<cv::KeyPoint>& keypoints1,
            const std::vector<cv::KeyPoint>& keypoints2,
            const cv::Mat& descriptors1,
            const cv::Mat& descriptors2
        );


    private:
        cv::Ptr<cv::BFMatcher> m_bfmatcher {};
        MatchParams2D m_params {};
    };

}   // namespace bot_vision::correspondence

#endif
