#ifndef CAMERA_CONTEXT_HPP
#define CAMERA_CONTEXT_HPP

#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core/mat.hpp>

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace bot_vision::camera_context
{
    struct CameraCalibration
    {   
        std::size_t m_id {};
        std::string m_frame_id {""};

        int m_img_height {0};
        int m_img_width {0};

        std::string m_distortion_model {""};
        std::vector<double> m_dist_params {};

        Eigen::Matrix3d m_K {};
        Eigen::Matrix4d m_Sigma_kappa {Eigen::Matrix4d::Zero()};

        Eigen::Matrix3d m_R {};
        Eigen::Matrix<double, 3, 4> m_P {};

        Eigen::Vector3d m_C {};
        Eigen::Matrix3d m_Sigma_C {Eigen::Matrix3d::Zero()};

        Eigen::Matrix3d m_Rot {};
        Eigen::Matrix3d m_Sigma_theta {Eigen::Matrix3d::Zero()};

        void populateCameraCalibration(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);

        void setCameraId(const std::size_t idx) {m_id = idx;}
        
        void setCameraExtrinsics(const geometry_msgs::msg::TransformStamped& tf_msg);
    };

    cv::Mat toCvMat(const Eigen::Matrix3d& eigen_mat);

    
}


#endif
