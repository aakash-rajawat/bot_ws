#include "bot_vision/preprocessing.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace bot_vision::preprocessing
{

RectificationMaps buildRectificationMaps(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr l_info,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr r_info
)
{
    cv::Mat l_K(3, 3, CV_64F);
    std::memcpy(l_K.data, l_info->k.data(), 9 * sizeof(double));

    cv::Mat l_d(1, static_cast<int>(l_info->d.size()), CV_64F);
    std::memcpy(l_d.data, l_info->d.data(), static_cast<int>(l_info->d.size()) * sizeof(double));

    cv::Size l_img_size(l_info->width, l_info->height);

    cv::Mat l_P(3, 4, CV_64F);
    std::memcpy(l_P.data, l_info->p.data(), 12 * sizeof(double));

    cv::Mat l_R(3, 3, CV_64F);
    std::memcpy(l_R.data, l_info->r.data(), 9 * sizeof(double));


    cv::Mat r_K(3, 3, CV_64F);
    std::memcpy(r_K.data, r_info->k.data(), 9 * sizeof(double));

    cv::Mat r_d(1, static_cast<int>(r_info->d.size()), CV_64F);
    std::memcpy(r_d.data, r_info->d.data(), static_cast<int>(r_info->d.size()) * sizeof(double));

    cv::Size r_img_size(r_info->width, r_info->height);

    cv::Mat r_P(3, 4, CV_64F);
    std::memcpy(r_P.data, r_info->p.data(), 12 * sizeof(double));

    cv::Mat r_R(3, 3, CV_64F);
    std::memcpy(r_R.data, r_info->r.data(), 9 * sizeof(double));

    RectificationMaps maps {};

    cv::initUndistortRectifyMap(
        l_K, 
        l_d, 
        l_R, 
        l_P, 
        l_img_size, 
        CV_32FC1, 
        maps.m_l_map1, 
        maps.m_l_map2
    );

    cv::initUndistortRectifyMap(
        r_K, 
        r_d, 
        r_R, 
        r_P, 
        r_img_size, 
        CV_32FC1, 
        maps.m_r_map1, 
        maps.m_r_map2
    );

    return maps;
}

void rectifyStereoPair(
    const cv::Mat& l_img_raw,
    const cv::Mat& r_img_raw,
    const RectificationMaps& maps,
    cv::Mat& l_rect,
    cv::Mat& r_rect
)
{
    cv::remap(
        l_img_raw, 
        l_rect,
        maps.m_l_map1,
        maps.m_l_map2,
        cv::INTER_LINEAR
    );

    cv::remap(
        r_img_raw,
        r_rect,
        maps.m_r_map1,
        maps.m_r_map2,
        cv::INTER_LINEAR
    );
}


} // namespace bot_vision
