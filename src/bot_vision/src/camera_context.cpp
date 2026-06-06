#include "bot_vision/camera_context.hpp"

#include <Eigen/Core>

namespace bot_vision::camera_context
{
    void CameraCalibration::populateCameraCalibration(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
    {
        m_frame_id = info_msg->header.frame_id;

        m_img_height = info_msg->height;
        m_img_width = info_msg->width;

        m_distortion_model = info_msg->distortion_model;
        m_dist_params = info_msg->d;

        m_K = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            info_msg->k.data()
        );

        m_R = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            info_msg->r.data()
        );

        m_P = Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(
            info_msg->p.data()
        );
    }

    void CameraCalibration::setCameraExtrinsics(
        const geometry_msgs::msg::TransformStamped& tf_msg
    )
    {
        m_C(0) = tf_msg.transform.translation.x;
        m_C(1) = tf_msg.transform.translation.y;
        m_C(2) = tf_msg.transform.translation.z;

        const auto& q_ros = tf_msg.transform.rotation;
        Eigen::Quaterniond q_eigen(q_ros.w, q_ros.x, q_ros.y, q_ros.z);
        q_eigen.normalize();
        m_Rot = q_eigen.toRotationMatrix();
    }

    cv::Mat toCvMat(const Eigen::Matrix3d& eigen_mat)
    {
        return (cv::Mat_<double>(3, 3) <<
            eigen_mat(0, 0), eigen_mat(0, 1), eigen_mat(0, 2),
            eigen_mat(1, 0), eigen_mat(1, 1), eigen_mat(1, 2),
            eigen_mat(2, 0), eigen_mat(2, 1), eigen_mat(2, 2)
        );
    }

}
