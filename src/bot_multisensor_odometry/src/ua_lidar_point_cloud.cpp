#include "bot_multisensor_odometry/ua_lidar_point_cloud.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <exception>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace bot_multisensor_odometry
{
    namespace
    {
        bool finitenessCheck(
            const Eigen::Vector3d& X_i,
            const Eigen::Matrix3d& Sigma_X_i
        )
        {
            return X_i.allFinite() && Sigma_X_i.allFinite();
        }

        bool symPosDefCheck(
            const Eigen::Matrix3d& Sigma_X_i,
            const double symmetric_tolerance = 1e-9,
            const double min_eigenvalue = 1.0e-4
        )
        {
            const bool symmetric =
                Sigma_X_i.isApprox(Sigma_X_i.transpose(), symmetric_tolerance);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_solver(Sigma_X_i);
            const bool eig_ok = (eig_solver.info() == Eigen::Success);

            const bool positive_definite =
                eig_ok &&
                (eig_solver.eigenvalues().array() >= min_eigenvalue).all();

            return symmetric && positive_definite;
        }

        Eigen::Matrix3d regularizeCovariance(
            const Eigen::Matrix3d& Sigma_X_i,
            const double min_variance
        )
        {
            const Eigen::Matrix3d Sigma_sym =
                0.5 * (Sigma_X_i + Sigma_X_i.transpose());

            if(min_variance <= 0.0)
            {
                return Sigma_sym;
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_solver(Sigma_sym);
            if(eig_solver.info() != Eigen::Success)
            {
                return Sigma_sym + min_variance * Eigen::Matrix3d::Identity();
            }

            const Eigen::Vector3d evals =
                eig_solver.eigenvalues().cwiseMax(min_variance);
            const Eigen::Matrix3d Sigma_reg =
                eig_solver.eigenvectors() * evals.asDiagonal() * eig_solver.eigenvectors().transpose();

            return 0.5 * (Sigma_reg + Sigma_reg.transpose());
        }
    }

    void LidarModel::setLidarIntrinsics(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        m_theta_min = scan_msg->angle_min;
        m_Delta_theta = scan_msg->angle_increment;
        m_theta_max = scan_msg->angle_max;
        m_time_inc = scan_msg->time_increment;
        m_range_min = scan_msg->range_min;
        m_range_max = scan_msg->range_max;
    }

    void LidarModel::setLidarExtrinsics(const geometry_msgs::msg::TransformStamped& tf_msg)
    {
        m_lidar_center(0) = tf_msg.transform.translation.x;
        m_lidar_center(1) = tf_msg.transform.translation.y;
        m_lidar_center(2) = tf_msg.transform.translation.z;

        const auto& q_ros = tf_msg.transform.rotation;
        Eigen::Quaterniond q_eigen(q_ros.w, q_ros.x, q_ros.y, q_ros.z);
        q_eigen.normalize();
        m_Rot = q_eigen.toRotationMatrix().transpose();
    }

    UALidarPointCloud::UALidarPointCloud(const std::string& name)
    : Node(name),
    m_lidar_precompute_ready(false),
    m_tf_buffer(this->get_clock()),
    m_tf_listener(m_tf_buffer)
    {
        declare_parameter<double>("covariance_min_stddev", 0.01);
        const double covariance_min_stddev =
            get_parameter("covariance_min_stddev").as_double();

        if(std::isfinite(covariance_min_stddev) && covariance_min_stddev > 0.0)
        {
            m_covariance_min_variance = covariance_min_stddev * covariance_min_stddev;
        }

        RCLCPP_INFO(
            get_logger(),
            "LiDAR covariance regularization: covariance_min_stddev=%.6f covariance_min_variance=%.6e",
            std::sqrt(m_covariance_min_variance),
            m_covariance_min_variance
        );

        rclcpp::QoS scan_sub_qos(10);
        m_scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            scan_sub_qos,
            std::bind(&UALidarPointCloud::scanCallback, this, std::placeholders::_1)
        );

        rclcpp::QoS pt_cloud_pub_qos(10);
        m_pt_cloud_pub = create_publisher<bot_interfaces::msg::PointWithCovarianceArray>(
            "/ua_lidar/ua_point_cloud",
            pt_cloud_pub_qos
        );

        rclcpp::QoS pts_pub_qos(10);
        m_pts_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/ua_lidar/point_cloud",
            pts_pub_qos
        );


    }
    void UALidarPointCloud::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        if(!m_lidar_precompute_ready)
        {
            m_lidar.setLidarIntrinsics(scan_msg);
            try
            {
                const auto tf_msg = m_tf_buffer.lookupTransform(
                    "base_footprint",
                    "lidar_link",
                    tf2::TimePointZero
                );

                m_lidar.setLidarExtrinsics(tf_msg);
                m_lidar_precompute = CasADi::runPerLidarPrecompute(m_lidar);
                m_lidar_precompute_ready = true;

                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "LiDAR ready."
                );
            }
            catch(const tf2::TransformException& ex)
            {
                m_lidar_precompute_ready = false;
                RCLCPP_WARN_STREAM(get_logger(), "tf lookup failed for tf frame: lidar_link"
                                                    << " with exception: " << ex.what());
                return;
            }
            catch(const std::exception& ex)
            {
                m_lidar_precompute_ready = false;
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "LiDAR precompute failed with exception: " << ex.what()
                );
                return;
            }
        }

        if(m_lidar_precompute_ready)
        {
            m_point_cloud.header.stamp = scan_msg->header.stamp;
            m_point_cloud.header.frame_id = "base_footprint";
            computeAndPublishPointCloud(scan_msg->ranges);
            return;
        }
    }

    void UALidarPointCloud::publish3DPoints(
        const bot_interfaces::msg::PointWithCovarianceArray& point_cloud
    )
    {
        const std::size_t point_count = std::min({
            point_cloud.x.size(),
            point_cloud.y.size(),
            point_cloud.z.size()
        });

        if(point_count == 0)
        {
            RCLCPP_DEBUG_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Skipping LiDAR PointCloud2 publish: zero points."
            );
            return;
        }

        if(
            point_cloud.x.size() != point_cloud.y.size() ||
            point_cloud.x.size() != point_cloud.z.size()
        )
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "LiDAR point cloud coordinate vector sizes differ: x=%zu y=%zu z=%zu; publishing %zu points.",
                point_cloud.x.size(),
                point_cloud.y.size(),
                point_cloud.z.size(),
                point_count
            );
        }

        sensor_msgs::msg::PointCloud2 cloud_msg {};
        cloud_msg.header = point_cloud.header;
        cloud_msg.height = 1;
        cloud_msg.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(point_count);

        sensor_msgs::PointCloud2Iterator<float> x_iter(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> y_iter(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> z_iter(cloud_msg, "z");

        for(std::size_t point_idx {0}; point_idx < point_count; ++point_idx)
        {
            *x_iter = point_cloud.x[point_idx];
            *y_iter = point_cloud.y[point_idx];
            *z_iter = point_cloud.z[point_idx];

            ++x_iter;
            ++y_iter;
            ++z_iter;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Publishing %zu LiDAR PointCloud2 points in frame=%s first_point=(%.3f, %.3f, %.3f)",
            point_count,
            cloud_msg.header.frame_id.c_str(),
            point_cloud.x.front(),
            point_cloud.y.front(),
            point_cloud.z.front()
        );

        m_pts_pub->publish(cloud_msg);
    }

    void UALidarPointCloud::computeAndPublishPointCloud(const std::vector<float>& ranges)
    {
        m_point_cloud.x.clear();
        m_point_cloud.y.clear();
        m_point_cloud.z.clear();
        m_point_cloud.covariance_xx.clear();
        m_point_cloud.covariance_xy.clear();
        m_point_cloud.covariance_xz.clear();
        m_point_cloud.covariance_yy.clear();
        m_point_cloud.covariance_yz.clear();
        m_point_cloud.covariance_zz.clear();

        m_point_cloud.x.reserve(ranges.size());
        m_point_cloud.y.reserve(ranges.size());
        m_point_cloud.z.reserve(ranges.size());
        m_point_cloud.covariance_xx.reserve(ranges.size());
        m_point_cloud.covariance_xy.reserve(ranges.size());
        m_point_cloud.covariance_xz.reserve(ranges.size());
        m_point_cloud.covariance_yy.reserve(ranges.size());
        m_point_cloud.covariance_yz.reserve(ranges.size());
        m_point_cloud.covariance_zz.reserve(ranges.size());

        for(std::size_t i = 0; i < ranges.size(); ++i)
        {
            const float rho_i = ranges[i];
            if(!CasADi::isValidRange(m_lidar, rho_i))
            {
                continue;
            }

            try
            {
                const auto beam = CasADi::runPerRangeCompute(
                    m_lidar_precompute,
                    i,
                    rho_i,
                    m_sigma_rho_i
                );

                auto regularized_beam = beam;
                regularized_beam.Sigma_X_i =
                    regularizeCovariance(beam.Sigma_X_i, m_covariance_min_variance);

                if(
                    !finitenessCheck(regularized_beam.X_i, regularized_beam.Sigma_X_i) ||
                    !symPosDefCheck(
                        regularized_beam.Sigma_X_i,
                        1e-9,
                        m_covariance_min_variance
                    )
                )
                {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        2000,
                        "Skipping LiDAR beam %zu due to invalid point/covariance output.",
                        i
                    );
                    continue;
                }

                const auto point = CasADi::beamToPointWithCovariance(regularized_beam);
                m_point_cloud.x.push_back(point.x);
                m_point_cloud.y.push_back(point.y);
                m_point_cloud.z.push_back(point.z);
                m_point_cloud.covariance_xx.push_back(point.covariance_xx);
                m_point_cloud.covariance_xy.push_back(point.covariance_xy);
                m_point_cloud.covariance_xz.push_back(point.covariance_xz);
                m_point_cloud.covariance_yy.push_back(point.covariance_yy);
                m_point_cloud.covariance_yz.push_back(point.covariance_yz);
                m_point_cloud.covariance_zz.push_back(point.covariance_zz);
            }
            catch(const std::exception& ex)
            {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "per_range_compute failed for beam " << i << ": " << ex.what()
                );
                continue;
            }
        }
        m_pt_cloud_pub->publish(m_point_cloud);
        publish3DPoints(m_point_cloud);
        return;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bot_multisensor_odometry::UALidarPointCloud>("ua_lidar_point_cloud"));
    rclcpp::shutdown();
    
    return 0;
}
