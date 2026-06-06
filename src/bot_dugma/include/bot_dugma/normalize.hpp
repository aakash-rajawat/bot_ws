#ifndef NORMALIZE_HPP
#define NORMALIZE_HPP

#include "bot_dugma/types.hpp"
#include <cmath>
#include <array>

namespace bot_dugma
{
    struct NormalizationInfo
    {
        std::array<float, 3> m_X_mean{0.0f, 0.0f, 0.0f};
        std::array<float, 3> m_Y_mean{0.0f, 0.0f, 0.0f};
        float m_scale {0.0f};
    };

    struct NormalizedPointClouds
    {
        PointCloud m_X {};
        PointCloud m_Y {};
        NormalizationInfo m_info {};

        void allocatePointCloud(
            const PointCloudView& in,
            PointCloud& out
        );

        void centerPointCloud(
            const PointCloudView& in, 
            PointCloud& out, 
            std::array<float, 3>& mean
        );

        float computeScale(const PointCloudView& in);

        void scalePointCloud(PointCloud& X);

        void normalizePointClouds(
            const PointCloudView& in_X,
            const PointCloudView& in_Y
        );
    };

    PoseEstimate denormalizePose(
        const PoseEstimate& normalized_pose,
        const NormalizationInfo& normalization_info
    );

    PoseEstimate normalizePose(
        const PoseEstimate& original_pose,
        const NormalizationInfo& normalization_info
    );
}

#endif
