#ifndef GTSAM_POSE_SLAM_HPP
#define GTSAM_POSE_SLAM_HPP

#include <bot_interfaces/msg/relative_pose_with_covariance_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>

class GtsamPoseSlam : public rclcpp::Node
{
public:
    explicit GtsamPoseSlam(const std::string& name);

private:
    using RelativePoseMsg = bot_interfaces::msg::RelativePoseWithCovarianceStamped;
    using Cov3 = Eigen::Matrix3d;

    void wheelCallback(const RelativePoseMsg::SharedPtr msg);
    void lidarCallback(const RelativePoseMsg::SharedPtr msg);
    void triangCallback(const RelativePoseMsg::SharedPtr msg);

    void processMeasurement(
        const RelativePoseMsg& msg,
        const std::string& source_name,
        bool robust
    );

    gtsam::Key getOrCreateKey(
        const builtin_interfaces::msg::Time& stamp,
        const std::optional<gtsam::Pose2>& initial_guess
    );
    std::optional<gtsam::Key> findNearbyExistingKey(int64_t ns) const;

    gtsam::Pose2 pose2FromMsg(const geometry_msgs::msg::Pose& pose) const;
    Cov3 planarCovarianceFromMsg(
        const geometry_msgs::msg::PoseWithCovariance& pose_cov
    ) const;

    bool isValidCovariance3(const Cov3& Q) const;
    Cov3 symmetrized(const Cov3& Q) const;

    Cov3 convertLeftCovarianceToGtsamPose2(
        const gtsam::Pose2& z,
        const Cov3& Q_left
    ) const;

    gtsam::noiseModel::Base::shared_ptr makeNoiseModel(
        const Cov3& Q,
        bool robust
    ) const;

    void addInitialPriorIfNeeded(const gtsam::Key& key);
    void updateIsamAndPublish();
    void publishLatestEstimate();

    static int64_t toNanoseconds(const builtin_interfaces::msg::Time& stamp);

    rclcpp::Subscription<RelativePoseMsg>::SharedPtr m_wheel_pose {};
    rclcpp::Subscription<RelativePoseMsg>::SharedPtr m_lidar_pose {};
    rclcpp::Subscription<RelativePoseMsg>::SharedPtr m_triang_pose {};

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_fused_odom_pub {};

    gtsam::ISAM2 m_isam {};
    gtsam::NonlinearFactorGraph m_new_factors {};
    gtsam::Values m_new_values {};

    std::map<int64_t, gtsam::Key> m_stamp_to_key {};
    std::map<gtsam::Key, int64_t> m_key_to_stamp {};

    std::optional<gtsam::Key> m_first_key {};
    std::optional<gtsam::Key> m_latest_key {};

    std::string m_world_frame_id {"odom"};
    std::string m_child_frame_id {"base_footprint_gtsam"};
    gtsam::Pose2 m_initial_pose {0.0, 0.0, 0.0};

    int64_t m_auxiliary_key_match_tolerance_ns {20000000};
    std::size_t m_next_index {0};
};

#endif
