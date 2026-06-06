#include "bot_localization/gtsam_pose_slam.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <Eigen/Eigenvalues>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <exception>
#include <functional>
#include <iterator>
#include <limits>

namespace
{
    Eigen::Matrix3d adjointPose2(const gtsam::Pose2& p)
    {
        const double x = p.x();
        const double y = p.y();
        const double th = p.theta();

        const double c = std::cos(th);
        const double s = std::sin(th);

        Eigen::Matrix3d Ad = Eigen::Matrix3d::Identity();
        Ad << c, -s, y,
              s,  c, -x,
              0.0, 0.0, 1.0;
        return Ad;
    }
}

GtsamPoseSlam::GtsamPoseSlam(const std::string& name)
    : Node(name)
{
    declare_parameter<std::string>(
        "topic_wheel_relative_pose",
        "/bot_controller/relative_pose_wheel"
    );
    declare_parameter<std::string>(
        "topic_lidar_relative_pose",
        "/bot_controller/relative_pose_mle_lidar"
    );
    declare_parameter<std::string>(
        "topic_triang_relative_pose",
        "/bot_controller/relative_pose_mle_triangulation"
    );
    declare_parameter<std::string>("world_frame_id", "odom");
    declare_parameter<std::string>("child_frame_id", "base_footprint_gtsam");
    declare_parameter<double>("initial_pose_x", 0.0);
    declare_parameter<double>("initial_pose_y", 0.0);
    declare_parameter<double>("initial_pose_yaw", 0.0);
    declare_parameter<double>("auxiliary_key_match_tolerance_sec", 0.02);

    const double auxiliary_key_match_tolerance_sec =
        get_parameter("auxiliary_key_match_tolerance_sec").as_double();
    m_auxiliary_key_match_tolerance_ns = static_cast<int64_t>(
        (auxiliary_key_match_tolerance_sec > 0.0 ? auxiliary_key_match_tolerance_sec : 0.0)
        * 1.0e9
    );

    m_world_frame_id = get_parameter("world_frame_id").as_string();
    m_child_frame_id = get_parameter("child_frame_id").as_string();
    m_initial_pose = gtsam::Pose2(
        get_parameter("initial_pose_x").as_double(),
        get_parameter("initial_pose_y").as_double(),
        get_parameter("initial_pose_yaw").as_double()
    );

    const auto wheel_topic = get_parameter("topic_wheel_relative_pose").as_string();
    const auto lidar_topic = get_parameter("topic_lidar_relative_pose").as_string();
    const auto triang_topic = get_parameter("topic_triang_relative_pose").as_string();

    rclcpp::QoS qos(50);
    m_wheel_pose = create_subscription<RelativePoseMsg>(
        wheel_topic,
        qos,
        std::bind(&GtsamPoseSlam::wheelCallback, this, std::placeholders::_1)
    );

    m_lidar_pose = create_subscription<RelativePoseMsg>(
        lidar_topic,
        qos,
        std::bind(&GtsamPoseSlam::lidarCallback, this, std::placeholders::_1)
    );

    m_triang_pose = create_subscription<RelativePoseMsg>(
        triang_topic,
        qos,
        std::bind(&GtsamPoseSlam::triangCallback, this, std::placeholders::_1)
    );

    m_fused_odom_pub = create_publisher<nav_msgs::msg::Odometry>(
        "/bot_controller/odom_gtsam_fused",
        10
    );
}

void GtsamPoseSlam::wheelCallback(const RelativePoseMsg::SharedPtr msg)
{
    processMeasurement(*msg, "wheel", false);
}

void GtsamPoseSlam::lidarCallback(const RelativePoseMsg::SharedPtr msg)
{
    processMeasurement(*msg, "lidar", true);
}

void GtsamPoseSlam::triangCallback(const RelativePoseMsg::SharedPtr msg)
{
    processMeasurement(*msg, "triangulation", true);
}

void GtsamPoseSlam::processMeasurement(
    const RelativePoseMsg& msg,
    const std::string& source_name,
    bool robust
)
{
    const int64_t from_ns = toNanoseconds(msg.from_stamp);
    const int64_t to_ns = toNanoseconds(msg.to_stamp);
    if(from_ns == 0 || to_ns == 0 || to_ns <= from_ns)
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting %s factor because timestamps are invalid: from=%ld to=%ld.",
            source_name.c_str(),
            static_cast<long>(from_ns),
            static_cast<long>(to_ns)
        );
        return;
    }

    const gtsam::Pose2 z = pose2FromMsg(msg.pose.pose);
    Cov3 Q = planarCovarianceFromMsg(msg.pose);
    if(!isValidCovariance3(Q))
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting %s factor because covariance is invalid.",
            source_name.c_str()
        );
        return;
    }

    if(source_name == "lidar" || source_name == "triangulation")
    {
        Q = convertLeftCovarianceToGtsamPose2(z, Q);
    }

    if(!isValidCovariance3(Q))
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting %s factor because converted covariance is invalid.",
            source_name.c_str()
        );
        return;
    }

    const bool attach_to_existing_timeline =
        source_name == "lidar" || source_name == "triangulation";

    gtsam::Key key_from {};
    if(attach_to_existing_timeline)
    {
        const auto nearby_key = findNearbyExistingKey(from_ns);
        if(!nearby_key)
        {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting %s factor because from_stamp=%ld has no existing graph key within %.3f ms.",
                source_name.c_str(),
                static_cast<long>(from_ns),
                static_cast<double>(m_auxiliary_key_match_tolerance_ns) / 1.0e6
            );
            return;
        }
        key_from = *nearby_key;
    }
    else
    {
        key_from = getOrCreateKey(msg.from_stamp, std::nullopt);
        addInitialPriorIfNeeded(key_from);
    }

    std::optional<gtsam::Pose2> to_initial;
    if(m_new_values.exists(key_from))
    {
        const auto from_pose = m_new_values.at<gtsam::Pose2>(key_from);
        to_initial = from_pose.compose(z);
    }
    else
    {
        try
        {
            const auto estimate = m_isam.calculateEstimate();
            if(estimate.exists(key_from))
            {
                const auto from_pose = estimate.at<gtsam::Pose2>(key_from);
                to_initial = from_pose.compose(z);
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_DEBUG(
                get_logger(),
                "Could not initialize %s to-pose from current estimate: %s",
                source_name.c_str(),
                e.what()
            );
        }
    }

    gtsam::Key key_to {};
    if(attach_to_existing_timeline)
    {
        const auto nearby_key = findNearbyExistingKey(to_ns);
        if(!nearby_key)
        {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting %s factor because to_stamp=%ld has no existing graph key within %.3f ms.",
                source_name.c_str(),
                static_cast<long>(to_ns),
                static_cast<double>(m_auxiliary_key_match_tolerance_ns) / 1.0e6
            );
            return;
        }
        key_to = *nearby_key;

        if(key_from == key_to)
        {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting %s factor because both endpoint stamps matched the same graph key.",
                source_name.c_str()
            );
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Matched %s factor stamps to existing graph keys: %ld -> %ld became %ld -> %ld",
            source_name.c_str(),
            static_cast<long>(from_ns),
            static_cast<long>(to_ns),
            static_cast<long>(m_key_to_stamp.at(key_from)),
            static_cast<long>(m_key_to_stamp.at(key_to))
        );
    }
    else
    {
        key_to = getOrCreateKey(msg.to_stamp, to_initial);
    }

    m_new_factors.add(gtsam::BetweenFactor<gtsam::Pose2>(
        key_from,
        key_to,
        z,
        makeNoiseModel(Q, robust)
    ));

    m_latest_key = key_to;

    RCLCPP_INFO(
        get_logger(),
        "Added %s factor: %ld -> %ld",
        source_name.c_str(),
        static_cast<long>(from_ns),
        static_cast<long>(to_ns)
    );

    updateIsamAndPublish();
}

gtsam::Key GtsamPoseSlam::getOrCreateKey(
    const builtin_interfaces::msg::Time& stamp,
    const std::optional<gtsam::Pose2>& initial_guess
)
{
    const int64_t ns = toNanoseconds(stamp);
    const auto it = m_stamp_to_key.find(ns);
    if(it != m_stamp_to_key.end())
    {
        return it->second;
    }

    const gtsam::Key key = gtsam::Symbol('x', m_next_index++);
    m_stamp_to_key[ns] = key;
    m_key_to_stamp[key] = ns;

    if(initial_guess)
    {
        m_new_values.insert(key, *initial_guess);
    }
    else if(!m_first_key && m_stamp_to_key.size() == 1)
    {
        m_new_values.insert(key, m_initial_pose);
    }
    else
    {
        m_new_values.insert(key, gtsam::Pose2(0.0, 0.0, 0.0));
    }
    return key;
}

std::optional<gtsam::Key> GtsamPoseSlam::findNearbyExistingKey(int64_t ns) const
{
    if(m_stamp_to_key.empty())
    {
        return std::nullopt;
    }

    const auto distance_ns = [](int64_t a, int64_t b) {
        return a > b ? a - b : b - a;
    };

    int64_t best_distance = std::numeric_limits<int64_t>::max();
    std::optional<gtsam::Key> best_key;

    const auto consider = [&](std::map<int64_t, gtsam::Key>::const_iterator it) {
        if(it == m_stamp_to_key.end())
        {
            return;
        }

        const int64_t d = distance_ns(ns, it->first);
        if(d < best_distance)
        {
            best_distance = d;
            best_key = it->second;
        }
    };

    const auto lower = m_stamp_to_key.lower_bound(ns);
    consider(lower);
    if(lower != m_stamp_to_key.begin())
    {
        consider(std::prev(lower));
    }

    if(best_key && best_distance <= m_auxiliary_key_match_tolerance_ns)
    {
        return best_key;
    }
    return std::nullopt;
}

gtsam::Pose2 GtsamPoseSlam::pose2FromMsg(
    const geometry_msgs::msg::Pose& pose
) const
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    q.normalize();

    double roll {};
    double pitch {};
    double yaw {};
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    return gtsam::Pose2(pose.position.x, pose.position.y, yaw);
}

GtsamPoseSlam::Cov3 GtsamPoseSlam::planarCovarianceFromMsg(
    const geometry_msgs::msg::PoseWithCovariance& pose_cov
) const
{
    const auto& c = pose_cov.covariance;

    Cov3 Q;
    Q << c[0], c[1], c[5],
         c[6], c[7], c[11],
         c[30], c[31], c[35];
    return symmetrized(Q);
}

bool GtsamPoseSlam::isValidCovariance3(const Cov3& Q) const
{
    if(!Q.allFinite())
    {
        return false;
    }

    const Cov3 Q_sym = symmetrized(Q);
    Eigen::SelfAdjointEigenSolver<Cov3> solver(Q_sym);
    if(solver.info() != Eigen::Success)
    {
        return false;
    }

    constexpr double kMinEig {1.0e-12};
    return solver.eigenvalues().minCoeff() > kMinEig;
}

GtsamPoseSlam::Cov3 GtsamPoseSlam::symmetrized(const Cov3& Q) const
{
    return 0.5 * (Q + Q.transpose());
}

GtsamPoseSlam::Cov3 GtsamPoseSlam::convertLeftCovarianceToGtsamPose2(
    const gtsam::Pose2& z,
    const Cov3& Q_left
) const
{
    const Cov3 Ad = adjointPose2(z.inverse());
    return symmetrized(Ad * Q_left * Ad.transpose());
}

gtsam::noiseModel::Base::shared_ptr GtsamPoseSlam::makeNoiseModel(
    const Cov3& Q,
    bool robust
) const
{
    auto gaussian = gtsam::noiseModel::Gaussian::Covariance(Q);
    if(!robust)
    {
        return gaussian;
    }

    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        gaussian
    );
}

void GtsamPoseSlam::addInitialPriorIfNeeded(const gtsam::Key& key)
{
    if(m_first_key)
    {
        return;
    }

    m_first_key = key;

    Cov3 prior_cov = Cov3::Zero();
    prior_cov(0, 0) = 1.0e-6;
    prior_cov(1, 1) = 1.0e-6;
    prior_cov(2, 2) = 1.0e-6;

    m_new_factors.add(gtsam::PriorFactor<gtsam::Pose2>(
        key,
        m_initial_pose,
        gtsam::noiseModel::Gaussian::Covariance(prior_cov)
    ));
}

void GtsamPoseSlam::updateIsamAndPublish()
{
    if(m_new_factors.size() == 0 && m_new_values.size() == 0)
    {
        return;
    }

    try
    {
        m_isam.update(m_new_factors, m_new_values);
        m_isam.update();

        m_new_factors = gtsam::NonlinearFactorGraph {};
        m_new_values = gtsam::Values {};

        publishLatestEstimate();
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "GTSAM update failed: %s", e.what());
        m_new_factors = gtsam::NonlinearFactorGraph {};
        m_new_values = gtsam::Values {};
    }
}

void GtsamPoseSlam::publishLatestEstimate()
{
    if(!m_latest_key)
    {
        return;
    }

    const gtsam::Values estimate = m_isam.calculateEstimate();
    if(!estimate.exists(*m_latest_key))
    {
        return;
    }

    const gtsam::Pose2 p = estimate.at<gtsam::Pose2>(*m_latest_key);

    nav_msgs::msg::Odometry odom {};
    odom.header.frame_id = m_world_frame_id;
    odom.child_frame_id = m_child_frame_id;

    const int64_t ns = m_key_to_stamp.at(*m_latest_key);
    odom.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
    odom.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);

    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q {};
    q.setRPY(0.0, 0.0, p.theta());
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.pose.covariance.fill(0.0);
    odom.pose.covariance[14] = 1.0e6;
    odom.pose.covariance[21] = 1.0e6;
    odom.pose.covariance[28] = 1.0e6;

    try
    {
        const auto P = m_isam.marginalCovariance(*m_latest_key);
        odom.pose.covariance[0] = P(0, 0);
        odom.pose.covariance[1] = P(0, 1);
        odom.pose.covariance[5] = P(0, 2);
        odom.pose.covariance[6] = P(1, 0);
        odom.pose.covariance[7] = P(1, 1);
        odom.pose.covariance[11] = P(1, 2);
        odom.pose.covariance[30] = P(2, 0);
        odom.pose.covariance[31] = P(2, 1);
        odom.pose.covariance[35] = P(2, 2);
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN(
            get_logger(),
            "Could not compute GTSAM marginal covariance: %s",
            e.what()
        );
    }

    m_fused_odom_pub->publish(odom);
}

int64_t GtsamPoseSlam::toNanoseconds(
    const builtin_interfaces::msg::Time& stamp
)
{
    return static_cast<int64_t>(stamp.sec) * 1000000000LL +
           static_cast<int64_t>(stamp.nanosec);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtsamPoseSlam>("gtsam_pose_slam"));
    rclcpp::shutdown();
    return 0;
}
