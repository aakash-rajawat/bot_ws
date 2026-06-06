#ifndef MULTIVIEW_POLICY_HPP
#define MULTIVIEW_POLICY_HPP

#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <cassert>

namespace bot_vision::multiview_policy
{
    struct DurationArray
    {
        std::vector<rclcpp::Duration> m_duration_array {};

        void pushBack(const rclcpp::Duration& value)
        {
            m_duration_array.push_back(value);
        }

        std::size_t size() const
        {
            return m_duration_array.size();
        }

        const rclcpp::Duration& operator[](const std::size_t idx) const
        {
            return m_duration_array[idx];
        }

        rclcpp::Duration& operator[](const std::size_t idx)
        {
            return m_duration_array[idx];
        }
    };

    struct TimestampArray
    {
        std::vector<rclcpp::Time> m_timestamp_array {};

        std::size_t size() const
        {
            return m_timestamp_array.size();
        }

        const rclcpp::Time& operator[](const std::size_t idx) const
        {
            return m_timestamp_array[idx];
        }

        rclcpp::Time& operator[](const std::size_t idx)
        {
            return m_timestamp_array[idx];
        }

        DurationArray difference(const TimestampArray& second) const
        {
            assert(m_timestamp_array.size() == second.size());

            DurationArray result {};
            for(std::size_t i {0}; i < m_timestamp_array.size(); ++i)
            {
                result.pushBack(m_timestamp_array[i] - second[i]);  
            }
            return result;
        }
    };

    struct DynamicReferenceCam
    {
        bool m_is_first {true};
        std::size_t m_ref_cam_idx {0};
        rclcpp::Duration m_max_time_diff{0, 50000000};
        TimestampArray m_current_timestamps {};
        std::vector<bool> m_result {};

        TimestampArray m_prev_timestamps {};
        std::size_t m_window {20};
        std::vector<DurationArray> m_interval_buffer {};
        std::vector<double> m_cam_means {};
        std::vector<double> m_new_cam_means {};
        std::vector<double> m_cam_vars {};
        std::vector<double> m_new_cam_vars {};


        void computeMean(const std::vector<DurationArray>& intervals);

        void computeVar(
            const std::vector<double>& cam_means,
            const std::vector<DurationArray>& intervals 
        );

        void updateMean(
            const DurationArray& new_interval,
            const DurationArray& oldest_interval
        );

        void updateVar(
            const DurationArray& new_interval,
            const DurationArray& oldest_interval
        );

        void getTimestamps(const TimestampArray& timestamps);

        void getClosestSet();

        void changeReferenceCam();
    };  // Dynamic Reference Cam

    

}

#endif
