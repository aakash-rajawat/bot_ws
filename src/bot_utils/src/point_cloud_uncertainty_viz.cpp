#include "bot_utils/point_cloud_uncertainty_viz.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <visualization_msgs/msg/marker.hpp>

namespace bot_utils
{
namespace
{

constexpr std::size_t kCoordinateAndCovarianceArrayCount {9};

bool hasConsistentArraySizes(const bot_interfaces::msg::PointWithCovarianceArray& cloud)
{
    const std::size_t point_count = cloud.x.size();
    const std::array<std::size_t, kCoordinateAndCovarianceArrayCount> sizes {
        cloud.x.size(),
        cloud.y.size(),
        cloud.z.size(),
        cloud.covariance_xx.size(),
        cloud.covariance_xy.size(),
        cloud.covariance_xz.size(),
        cloud.covariance_yy.size(),
        cloud.covariance_yz.size(),
        cloud.covariance_zz.size()
    };

    return std::all_of(
        sizes.begin(),
        sizes.end(),
        [point_count](const std::size_t size) {return size == point_count;}
    );
}

std_msgs::msg::ColorRGBA makeColor(
    const double red,
    const double green,
    const double blue,
    const double alpha)
{
    std_msgs::msg::ColorRGBA color;
    color.r = static_cast<float>(red);
    color.g = static_cast<float>(green);
    color.b = static_cast<float>(blue);
    color.a = static_cast<float>(alpha);
    return color;
}

}  // namespace

PointCloudUncertaintyViz::PointCloudUncertaintyViz(const std::string& name)
: Node(name)
{
    const std::string camera_input_topic = declare_parameter<std::string>(
        "camera_input_topic",
        "/ua_triangulation/pointswithcovariance"
    );
    const std::string lidar_input_topic = declare_parameter<std::string>(
        "lidar_input_topic",
        "/ua_lidar/ua_point_cloud"
    );
    const std::string camera_output_topic = declare_parameter<std::string>(
        "camera_output_topic",
        "/uncertainty_visualization/camera_ellipsoids"
    );
    const std::string lidar_output_topic = declare_parameter<std::string>(
        "lidar_output_topic",
        "/uncertainty_visualization/lidar_ellipsoids"
    );

    m_confidence_scale = declare_parameter<double>("confidence_scale", 2.795483482);
    m_magnification = declare_parameter<double>("magnification", 1.0);

    const auto camera_max_markers = declare_parameter<std::int64_t>("camera_max_markers", 0);
    const auto lidar_max_markers = declare_parameter<std::int64_t>("lidar_max_markers", 0);

    if(!std::isfinite(m_confidence_scale) || m_confidence_scale <= 0.0)
    {
        throw std::invalid_argument("confidence_scale must be finite and greater than zero");
    }
    if(!std::isfinite(m_magnification) || m_magnification <= 0.0)
    {
        throw std::invalid_argument("magnification must be finite and greater than zero");
    }
    if(camera_max_markers < 0 || lidar_max_markers < 0)
    {
        throw std::invalid_argument("marker limits must be non-negative; zero means unlimited");
    }

    m_camera_visualization.marker_namespace = "camera_point_uncertainty";
    m_camera_visualization.color = makeColor(
        declare_parameter<double>("camera_color.r", 0.10),
        declare_parameter<double>("camera_color.g", 0.75),
        declare_parameter<double>("camera_color.b", 1.00),
        declare_parameter<double>("camera_color.a", 0.35)
    );
    m_camera_visualization.max_markers = static_cast<std::size_t>(camera_max_markers);
    m_camera_visualization.publisher =
        create_publisher<visualization_msgs::msg::MarkerArray>(camera_output_topic, 10);

    m_lidar_visualization.marker_namespace = "lidar_point_uncertainty";
    m_lidar_visualization.color = makeColor(
        declare_parameter<double>("lidar_color.r", 1.00),
        declare_parameter<double>("lidar_color.g", 0.45),
        declare_parameter<double>("lidar_color.b", 0.05),
        declare_parameter<double>("lidar_color.a", 0.30)
    );
    m_lidar_visualization.max_markers = static_cast<std::size_t>(lidar_max_markers);
    m_lidar_visualization.publisher =
        create_publisher<visualization_msgs::msg::MarkerArray>(lidar_output_topic, 10);

    m_camera_subscription = create_subscription<PointCloud>(
        camera_input_topic,
        10,
        std::bind(&PointCloudUncertaintyViz::cameraCloudCallback, this, std::placeholders::_1)
    );
    m_lidar_subscription = create_subscription<PointCloud>(
        lidar_input_topic,
        10,
        std::bind(&PointCloudUncertaintyViz::lidarCloudCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
        get_logger(),
        "Point covariance visualization ready: confidence_scale=%.6f magnification=%.3f",
        m_confidence_scale,
        m_magnification
    );
}

void PointCloudUncertaintyViz::cameraCloudCallback(const PointCloud::ConstSharedPtr msg)
{
    publishEllipsoids(*msg, m_camera_visualization);
}

void PointCloudUncertaintyViz::lidarCloudCallback(const PointCloud::ConstSharedPtr msg)
{
    publishEllipsoids(*msg, m_lidar_visualization);
}

void PointCloudUncertaintyViz::publishEllipsoids(
    const PointCloud& cloud,
    const CloudVisualization& visualization)
{
    if(!hasConsistentArraySizes(cloud))
    {
        RCLCPP_WARN(
            get_logger(),
            "Skipping %s cloud because its coordinate and covariance array sizes differ.",
            visualization.marker_namespace.c_str()
        );
        return;
    }

    const std::size_t point_count = cloud.x.size();
    const std::size_t marker_count = visualization.max_markers == 0 ?
        point_count : std::min(point_count, visualization.max_markers);
    const double sampling_stride = marker_count == 0 ? 1.0 :
        static_cast<double>(point_count) / static_cast<double>(marker_count);

    visualization_msgs::msg::MarkerArray output;
    output.markers.reserve(marker_count + 1);

    visualization_msgs::msg::Marker delete_previous;
    delete_previous.header = cloud.header;
    delete_previous.action = visualization_msgs::msg::Marker::DELETEALL;
    output.markers.push_back(std::move(delete_previous));

    std::size_t rejected_points {0};
    for(std::size_t marker_index {0}; marker_index < marker_count; ++marker_index)
    {
        const std::size_t point_index = std::min(
            static_cast<std::size_t>(std::floor(marker_index * sampling_stride)),
            point_count - 1
        );

        const Eigen::Vector3d position(
            cloud.x[point_index],
            cloud.y[point_index],
            cloud.z[point_index]
        );
        Eigen::Matrix3d covariance;
        covariance <<
            cloud.covariance_xx[point_index], cloud.covariance_xy[point_index], cloud.covariance_xz[point_index],
            cloud.covariance_xy[point_index], cloud.covariance_yy[point_index], cloud.covariance_yz[point_index],
            cloud.covariance_xz[point_index], cloud.covariance_yz[point_index], cloud.covariance_zz[point_index];

        if(!position.allFinite() || !covariance.allFinite())
        {
            ++rejected_points;
            continue;
        }

        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
        if(eigensolver.info() != Eigen::Success || (eigensolver.eigenvalues().array() <= 0.0).any())
        {
            ++rejected_points;
            continue;
        }

        Eigen::Matrix3d orientation = eigensolver.eigenvectors();
        if(orientation.determinant() < 0.0)
        {
            orientation.col(2) *= -1.0;
        }
        Eigen::Quaterniond quaternion(orientation);
        quaternion.normalize();

        const Eigen::Vector3d diameters =
            2.0 * m_confidence_scale * m_magnification *
            eigensolver.eigenvalues().cwiseSqrt();

        visualization_msgs::msg::Marker marker;
        marker.header = cloud.header;
        marker.ns = visualization.marker_namespace;
        marker.id = static_cast<std::int32_t>(marker_index);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();
        marker.scale.x = diameters.x();
        marker.scale.y = diameters.y();
        marker.scale.z = diameters.z();
        marker.color = visualization.color;
        output.markers.push_back(std::move(marker));
    }

    visualization.publisher->publish(output);

    if(rejected_points > 0)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Rejected %zu invalid covariance ellipsoids from %s.",
            rejected_points,
            visualization.marker_namespace.c_str()
        );
    }
}

}  // namespace bot_utils

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<bot_utils::PointCloudUncertaintyViz>("point_cloud_uncertainty_viz")
    );
    rclcpp::shutdown();
    return 0;
}
