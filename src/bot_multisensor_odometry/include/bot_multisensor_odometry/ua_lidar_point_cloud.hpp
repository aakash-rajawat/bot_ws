#ifndef UA_LIDAR_POINT_CLOUD_HPP
#define UA_LIDAR_POINT_CLOUD_HPP


#include <bot_interfaces/msg/point_with_covariance_array.hpp>
#include "bot_multisensor_odometry/casadi_helper.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include <Eigen/Dense>

namespace bot_multisensor_odometry
{
    struct LidarModel
    {
        double m_theta_min {};
        double m_Delta_theta {};
        double m_theta_max {};
        double m_time_inc {};
        double m_range_min {};
        double m_range_max {};

        double m_b_rho {0.0};
        double m_sigma_b_rho {0.002};
        double m_s_rho {1.0};
        double m_sigma_s_rho {1.0e-3};
        double m_b_theta {0.0};
        double m_sigma_b_theta {1.0e-3};

        Eigen::Vector3d m_lidar_center {};
        double m_sigma_c_x {0.002};
        double m_sigma_c_y {0.002};
        double m_sigma_c_z {0.002};

        Eigen::Matrix3d m_Rot {};
        double m_sigma_phi_x {1.0e-3};
        double m_sigma_phi_y {1.0e-3};
        double m_sigma_phi_z {1.0e-3};

        void setLidarIntrinsics(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
        void setLidarExtrinsics(const geometry_msgs::msg::TransformStamped& tf_msg);
    };

    class UALidarPointCloud : public rclcpp::Node
    {
        public:
        UALidarPointCloud(const std::string& name);

        private:
        void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
        void computeAndPublishPointCloud(const std::vector<float>& ranges);
        void publish3DPoints(
            const bot_interfaces::msg::PointWithCovarianceArray& point_cloud
        );

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_sub {};
        rclcpp::Publisher<bot_interfaces::msg::PointWithCovarianceArray>::SharedPtr m_pt_cloud_pub {};
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pts_pub {};

        LidarModel m_lidar {};
        const double m_sigma_rho_i {0.01};
        double m_covariance_min_variance {1.0e-4};
        CasADi::PerLidarPrecompute m_lidar_precompute {};
        bool m_lidar_precompute_ready {};

        tf2_ros::Buffer m_tf_buffer;
        tf2_ros::TransformListener m_tf_listener;

        bot_interfaces::msg::PointWithCovarianceArray m_point_cloud {};
    };

}




#endif
