#ifndef FEATURE_EXTRACTION_HPP
#define FEATURE_EXTRACTION_HPP

#include <opencv2/features2d.hpp>
#include <opencv2/core/mat.hpp>

#include <vector>

namespace bot_vision::feature_extraction
{

    class ORBFeatureExtractor
    {
    public:
        ORBFeatureExtractor();
        void detectAndCompute(
            const cv::Mat& img,
            std::vector<cv::KeyPoint>& keypoints,
            cv::Mat& descriptors
        ) const;

    private:
        cv::Ptr<cv::ORB> m_detector {};

    };



}   // namespace bot_vision::feature_extraction


#endif
