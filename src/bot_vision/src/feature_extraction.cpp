#include "bot_vision/feature_extraction.hpp"

namespace bot_vision::feature_extraction
{
    ORBFeatureExtractor::ORBFeatureExtractor()
    {
        m_detector = cv::ORB::create();
    }

    void ORBFeatureExtractor::detectAndCompute(
        const cv::Mat& img,
        std::vector<cv::KeyPoint>& keypoints,
        cv::Mat& descriptors
    ) const
    {
        m_detector->detectAndCompute(
            img,
            cv::noArray(),
            keypoints,
            descriptors
        );
    }

}    // namespace bot_vision::feature_extraction
