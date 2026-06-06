#ifndef MLE_RELATIVE_POSE_CLIENT_HPP
#define MLE_RELATIVE_POSE_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <bot_interfaces/msg/point_with_covariance_array.hpp>
#include <bot_interfaces/action/mle_relative_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <bot_interfaces/msg/relative_pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/cache.hpp>

#include <memory>
#include <optional>

namespace bot_multisensor_odometry::mle_relative_pose_client
{
    class MLERelativePoseClient : public rclcpp::Node
    {
    public:
        explicit MLERelativePoseClient(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );

    private:
        void pointCloudCallback(
            const bot_interfaces::msg::PointWithCovarianceArray::ConstSharedPtr pt_cloud_msg
        );

        void maybeDispatchPendingCloud();

        void sendGoalMessage(
            const bot_interfaces::msg::PointWithCovarianceArray& x0,
            const bot_interfaces::msg::PointWithCovarianceArray& y0
        );

        // ROS Action client functons
        void goalCallback(
            const rclcpp_action::ClientGoalHandle<
                bot_interfaces::action::MLERelativePose
            >::SharedPtr& goal_handle
        );

        void feedbackCallback(
            rclcpp_action::ClientGoalHandle<
                bot_interfaces::action::MLERelativePose
            >::SharedPtr,
            const std::shared_ptr<
                const bot_interfaces::action::MLERelativePose::Feedback
            > feedback
        );

        void resultCallback(
            const rclcpp_action::ClientGoalHandle<
                bot_interfaces::action::MLERelativePose
            >::WrappedResult result
        );

        std::string m_topic_pt_cloud {""};
        std::string m_topic_relative_pose {""};

        rclcpp::Subscription<bot_interfaces::msg::PointWithCovarianceArray>::SharedPtr
            m_pt_cloud_sub {};

        std::string m_relative_pose_action_name {""};
        rclcpp_action::Client<bot_interfaces::action::MLERelativePose>::SharedPtr
            m_relative_pose_client {};

        

        bot_interfaces::msg::PointWithCovarianceArray::ConstSharedPtr
            m_previous_cloud {};

        bot_interfaces::msg::PointWithCovarianceArray::ConstSharedPtr
            m_pending_cloud {};
            
        std::optional<rclcpp::Time> m_previous_stamp {};
        rclcpp::Duration m_association_tolerance {0, 500000000};
        bool m_goal_in_flight {false};
        bool m_skip_stationary_cloud_pairs {true};
        double m_stationary_translation_threshold {0.01};
        double m_stationary_yaw_threshold {0.017453292519943295};

        

        rclcpp::Publisher<bot_interfaces::msg::RelativePoseWithCovarianceStamped>::SharedPtr
            m_relative_pose_pub {};

        message_filters::Subscriber<geometry_msgs::msg::PoseStamped> m_wheel_odom_sub {};
        unsigned int cache_len {200};
        message_filters::Cache<geometry_msgs::msg::PoseStamped>  m_wheel_odom_cache {
            m_wheel_odom_sub,
            cache_len
        };

        



    };
}

#endif
