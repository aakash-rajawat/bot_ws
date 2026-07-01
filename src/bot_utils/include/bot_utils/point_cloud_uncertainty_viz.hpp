#ifndef BOT_UTILS__POINT_CLOUD_UNCERTAINTY_VIZ_HPP_
#define BOT_UTILS__POINT_CLOUD_UNCERTAINTY_VIZ_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include <bot_interfaces/msg/point_with_covariance_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace bot_utils
{

class PointCloudUncertaintyViz : public rclcpp::Node
{
public:
    explicit PointCloudUncertaintyViz(const std::string& name);

private:
    using PointCloud = bot_interfaces::msg::PointWithCovarianceArray;

    struct CloudVisualization
    {
        std::string marker_namespace;
        std_msgs::msg::ColorRGBA color;
        std::size_t max_markers {0};
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher;
    };

    void cameraCloudCallback(const PointCloud::ConstSharedPtr msg);
    void lidarCloudCallback(const PointCloud::ConstSharedPtr msg);
    void gtsamPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void publishEllipsoids(const PointCloud& cloud, const CloudVisualization& visualization);

    // sqrt(chi-square quantile for three dimensions at 95% confidence).
    double m_confidence_scale {2.795483482};
    double m_magnification {1.0};

    // sqrt(chi-square quantile for two dimensions at 95% confidence).
    double m_gtsam_confidence_scale {2.447746831};
    double m_gtsam_magnification {1.0};
    double m_gtsam_ellipse_thickness {0.02};
    double m_gtsam_z_offset {0.03};
    std_msgs::msg::ColorRGBA m_gtsam_color;

    CloudVisualization m_camera_visualization;
    CloudVisualization m_lidar_visualization;

    rclcpp::Subscription<PointCloud>::SharedPtr m_camera_subscription;
    rclcpp::Subscription<PointCloud>::SharedPtr m_lidar_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_gtsam_subscription;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_gtsam_publisher;
};

}  // namespace bot_utils

#endif  // BOT_UTILS__POINT_CLOUD_UNCERTAINTY_VIZ_HPP_
