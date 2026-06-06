#ifndef VISION_FRONTEND_HPP
#define VISION_FRONTEND_HPP

#include "bot_vision/preprocessing.hpp"
#include "bot_vision/feature_extraction.hpp"
#include "bot_vision/correspondence.hpp"
#include "bot_vision/camera_context.hpp"
#include "bot_vision/multiview_policy.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <bot_interfaces/msg/point_with_covariance_array.hpp>
#include <bot_interfaces/srv/matched_correspondences2d.hpp>
#include <opencv2/core/mat.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <memory>
#include <vector>
#include <Eigen/Dense>


namespace bot_vision
{
    class VisionFrontend : public rclcpp::Node
    {
    public:
        explicit VisionFrontend(const std::string& name);

    private:

        void cameraInfoCallback(
            std::size_t cam_idx,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg
        );

        void imageArrivalCallback(
            std::size_t img_sub_idx,
            const sensor_msgs::msg::Image::ConstSharedPtr img_msg
        );

        void processSelectedViews();

        void matcherResponseCallback(
            rclcpp::Client<bot_interfaces::srv::MatchedCorrespondences2d>::SharedFuture future
        );

        void publish3DPoints(
            bot_interfaces::msg::PointWithCovarianceArray triangulated_points,
            double time
        );

        std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> m_img_subs {};
        std::vector<std::string> m_img_topics {""};
        std::vector<cv::Mat> m_raw_images {};

        std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> m_cam_info_subs {};
        std::vector<std::string> m_cam_info_topics {""};
        std::vector<camera_context::CameraCalibration> m_info_cameras {};
        std::vector<bool> m_camera_ready {};
        tf2_ros::Buffer m_tf_buffer;
        tf2_ros::TransformListener m_tf_listener;
        std::vector<std::string> m_cam_tf_names {""};

        rclcpp::Publisher<bot_interfaces::msg::PointWithCovarianceArray>::SharedPtr m_pts_cov_pub {};
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pts_pub {};
        

        std::vector<bool> m_has_timestamp{};
        multiview_policy::TimestampArray m_img_timestamps {};

        multiview_policy::DynamicReferenceCam m_id_selector {};
        std::vector<bool> m_selected_ids {};

        rclcpp::Client<bot_interfaces::srv::MatchedCorrespondences2d>::SharedPtr m_matcher_client {};
        bool m_matcher_request_in_flight {false};
        std::vector<std::size_t> m_pending_selected_cam_ids {};
        double m_pending_sigma_x {0.1};
        double m_pending_sigma_y {0.1};

        

        preprocessing::RectificationMaps m_rect_maps {};
        bool m_rect_maps_ready {};

        feature_extraction::ORBFeatureExtractor m_detector {};

        correspondence::MatchParams2D m_params = []()
        {
            correspondence::MatchParams2D params {};
            params.norm_type = cv::NORM_HAMMING;
            params.use_ratio_test = true;
            params.ratio_test_threshold = 0.72;
            params.use_descriptor_distance_filter = true;
            params.max_descriptor_distance = 40.0;
            params.use_mutual_consistency = true;
            params.use_fundamental_ransac = false;
            params.ransac_reproj_threshold = 1.5;
            params.ransac_confidence = 0.995;
            return params;
        }();
        correspondence::BFMatcherStereo m_bf_matcher{m_params};

    };
}


#endif
