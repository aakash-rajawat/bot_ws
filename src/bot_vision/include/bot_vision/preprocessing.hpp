#ifndef PREPROCESSING_HPP
#define PREPROCESSING_HPP

#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


namespace bot_vision::preprocessing
{

struct RectificationMaps
{
    cv::Mat m_l_map1 {};
    cv::Mat m_l_map2 {};
    cv::Mat m_r_map1 {};
    cv::Mat m_r_map2 {};
};

RectificationMaps buildRectificationMaps(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr l_info,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr r_info
);

void rectifyStereoPair(
    const cv::Mat& l_img_raw,
    const cv::Mat& r_img_raw,
    const RectificationMaps& maps,
    cv::Mat& l_rect,
    cv::Mat& r_rect
);


} // namespace bot_vision

#endif
