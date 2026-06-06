#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <string>
#include <cstddef>
#include <array>
#include <string_view>

namespace bot_dugma
{
    template<typename T>
    struct ArrayView
    {
        const T* m_data {nullptr};
        std::size_t m_size {0};

        ArrayView() = default;

        ArrayView(const std::vector<T>& values)
        : m_data(values.data()),
          m_size(values.size())
        {
        }

        const T& operator[](std::size_t idx) const
        {
            return m_data[idx];
        }

        std::size_t size() const
        {
            return m_size;
        }

        const T* begin() const
        {
            return m_data;
        }

        const T* end() const
        {
            return m_data + m_size;
        }
    };

    struct PointCloud
    {
        std::vector<float> m_x {}, m_y {}, m_z {};
        std::vector<float> m_Sxx {}, m_Sxy {}, m_Sxz {}, m_Syy {}, m_Syz {}, m_Szz {};

        double m_timestamp_sec {0.0};
        std::string m_frame_id {""};
    };

    struct PointCloudView
    {
        ArrayView<float> m_x {}, m_y {}, m_z {};
        ArrayView<float> m_Sxx {}, m_Sxy {}, m_Sxz {}, m_Syy {}, m_Syz {}, m_Szz {};

        double m_timestamp_sec {0.0};
        std::string_view m_frame_id {""};
    };

    PointCloudView makePointCloudView(const PointCloud& pt_cloud);

    struct PoseEstimate
    {
        float m_R[9] {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f
        };
        float m_t[3] {0.0f, 0.0f, 0.0f};
        bool m_is_converged {false};
        int m_iter {0};
        float m_energy {0.0f};
        std::array<float, 21> m_pose_cov {};
        bool m_has_pose_cov {false};
    };

    struct DugmaRegistrarConfig
    {
        std::size_t m_max_iter {100};
        float m_acc_tol_rotation {1e-4f};
        float m_acc_tol_translatation {1e-6f};
        float m_acc_tol_sigma {1e-8f};
        std::size_t m_max_pt_count {2048};
    };
    
}

#endif
