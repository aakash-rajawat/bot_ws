#include "bot_vision/multiview_policy.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <cassert>


namespace bot_vision::multiview_policy
{
    void DynamicReferenceCam::computeMean(const std::vector<DurationArray>& intervals)
        {
            for(std::size_t i {0}; i < m_current_timestamps.size(); ++i)
            {
                double sum {};
                for(std::size_t j {0}; j < intervals.size(); ++j)
                {
                    sum += intervals[j][i].seconds();
                }
                m_cam_means[i] = sum / static_cast<double>(intervals.size());
            }
        }

        void DynamicReferenceCam::computeVar(
            const std::vector<double>& cam_means,
            const std::vector<DurationArray>& intervals 
        )
        {
            assert(intervals.size() >= 2);

            for(std::size_t i {0}; i < m_current_timestamps.size(); ++i)
            {
                double var {};
                for(std::size_t j {0}; j < intervals.size(); ++j)
                {
                    var += (intervals[j][i].seconds() - cam_means[i])
                    * (intervals[j][i].seconds() - cam_means[i]);
                }
                m_cam_vars[i] = var / (static_cast<double>(intervals.size()) - 1);
            }
        }

        void DynamicReferenceCam::updateMean(
            const DurationArray& new_interval,
            const DurationArray& oldest_interval
        )
        {
            for(std::size_t i {0}; i < new_interval.size(); ++i)
            {
                m_new_cam_means[i] = m_cam_means[i] +
                                    (new_interval[i].seconds() - oldest_interval[i].seconds()) 
                                    / static_cast<double>(m_window);
            }
        }

        void DynamicReferenceCam::updateVar(
            const DurationArray& new_interval,
            const DurationArray& oldest_interval
        )
        {
            for(std::size_t i {0}; i < new_interval.size(); ++i)
            {
                m_new_cam_vars[i] = 
                    m_cam_vars[i] + (
                        (
                            new_interval[i].seconds() * new_interval[i].seconds() 
                            - oldest_interval[i].seconds() * oldest_interval[i].seconds()
                        ) 
                        / static_cast<double>(m_window)
                    )
                    - (
                        m_new_cam_means[i] * m_new_cam_means[i]
                        - m_cam_means[i] * m_cam_means[i]
                    );
                m_new_cam_vars[i] = std::max(0.0, m_new_cam_vars[i]);
            }
        }

        void DynamicReferenceCam::getTimestamps(const TimestampArray& timestamps)
        {
            m_current_timestamps = timestamps;
            m_result.resize(m_current_timestamps.size());
            if (m_cam_means.size() != m_current_timestamps.size()) 
            {
                m_cam_means.assign(m_current_timestamps.size(), 0.0);
                m_new_cam_means.assign(m_current_timestamps.size(), 0.0);
                m_cam_vars.assign(m_current_timestamps.size(), 0.0);
                m_new_cam_vars.assign(m_current_timestamps.size(), 0.0);
            }

            getClosestSet();

            m_is_first = false;
        }

        void DynamicReferenceCam::getClosestSet()
        {
            m_result.assign(m_current_timestamps.size(), false);
            m_result[m_ref_cam_idx] = true;

            for(std::size_t idx {0}; idx < m_current_timestamps.size(); ++idx)
            {
                if(idx == m_ref_cam_idx) continue;

                rclcpp::Duration diff = m_current_timestamps[idx] > m_current_timestamps[m_ref_cam_idx] ?
                    (m_current_timestamps[idx] - m_current_timestamps[m_ref_cam_idx])
                    : (m_current_timestamps[m_ref_cam_idx] - m_current_timestamps[idx]);

                if(diff <= m_max_time_diff)
                {
                    m_result[idx] = true;
                }
            }

            if(!m_is_first)
            {
                changeReferenceCam();
            }
            m_prev_timestamps = m_current_timestamps;
        }



        void DynamicReferenceCam::changeReferenceCam()
        {
            // keep track of the variance of the time intervals of each camera
            // the camera with the least variance is the most reliable
            // and shall be chosen as the reference camera

            auto interval = m_current_timestamps.difference(m_prev_timestamps);

            if (m_interval_buffer.size() < m_window - 1) 
            {
                m_interval_buffer.push_back(interval);
                return;
            }

            if(m_interval_buffer.size() == m_window - 1)
            {
                m_interval_buffer.push_back(interval);

                computeMean(m_interval_buffer);
                computeVar(m_cam_means, m_interval_buffer);

                auto it = std::min_element(m_cam_vars.begin(), m_cam_vars.end());
                m_ref_cam_idx = std::distance(m_cam_vars.begin(), it);
                return;
            }
            
            DurationArray oldest_interval = m_interval_buffer.front();
            m_interval_buffer.erase(m_interval_buffer.begin());
            m_interval_buffer.push_back(interval);
            
            updateMean(m_interval_buffer.back(), oldest_interval);
            updateVar(m_interval_buffer.back(), oldest_interval);

            auto it = std::min_element(m_new_cam_vars.begin(), m_new_cam_vars.end());
            m_ref_cam_idx = std::distance(m_new_cam_vars.begin(), it);

            m_cam_means = m_new_cam_means;
            m_cam_vars = m_new_cam_vars;
              
        }
}   // namespace bot_vision::multiview_policy
