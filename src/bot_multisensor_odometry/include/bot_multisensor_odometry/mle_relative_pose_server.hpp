#ifndef MLE_RELATIVE_POSE_SERVER_HPP
#define MLE_RELATIVE_POSE_SERVER_HPP

#include "bot_dugma/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <bot_interfaces/action/mle_relative_pose.hpp>

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <optional>
#include <memory>


namespace bot_multisensor_odometry::mle_relative_pose_server
{
    class MLERelativePoseServer : public rclcpp::Node
    {
    public:
        explicit MLERelativePoseServer(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
        ~MLERelativePoseServer() override;

    private:
        rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID& goal_uuid,
            std::shared_ptr<
                const bot_interfaces::action::MLERelativePose::Goal
            > goal
        );

        rclcpp_action::CancelResponse cancelCallback(
            const std::shared_ptr<
                rclcpp_action::ServerGoalHandle<bot_interfaces::action::MLERelativePose>
            > goal_handle
        );

        void acceptedCallback(
            const std::shared_ptr<
                rclcpp_action::ServerGoalHandle<bot_interfaces::action::MLERelativePose>
            > goal_handle
        );

        void workerLoop();

        void estimatePose(
            const std::shared_ptr<
                rclcpp_action::ServerGoalHandle<bot_interfaces::action::MLERelativePose>
            > goal_handle
        );

        void completeGoal(
            const bot_dugma::PoseEstimate& pose_estimate,
            const std::shared_ptr<
                rclcpp_action::ServerGoalHandle<bot_interfaces::action::MLERelativePose>
            > goal_handle
        );

        std::string m_relative_pose_action_name {""};
        rclcpp_action::Server<bot_interfaces::action::MLERelativePose>::SharedPtr
            m_relative_pose_server {};

        std::deque<
            std::shared_ptr<
                rclcpp_action::ServerGoalHandle<bot_interfaces::action::MLERelativePose>
            >
        > m_def_goal_handles {};

        std::mutex m_def_goal_handles_mutex {};
        std::condition_variable m_def_goal_handles_cond_var {};
        std::thread m_worker_thread {} ;
        bool m_stop_worker {false};

        std::size_t m_max_def_goals {2};

        bot_dugma::DugmaRegistrarConfig m_registrar_config {};
    };
}

#endif
