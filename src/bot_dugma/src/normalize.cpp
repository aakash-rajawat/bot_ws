#include "bot_dugma/normalize.hpp"

#include <cstdio>

namespace bot_dugma
{
    void NormalizedPointClouds::allocatePointCloud(
        const PointCloudView& in,
        PointCloud& out
    )
    {
        out.m_x.resize(in.m_x.size());
        out.m_y.resize(in.m_y.size());
        out.m_z.resize(in.m_z.size());

        out.m_Sxx.assign(in.m_Sxx.begin(), in.m_Sxx.end());
        out.m_Sxy.assign(in.m_Sxy.begin(), in.m_Sxy.end());
        out.m_Sxz.assign(in.m_Sxz.begin(), in.m_Sxz.end());
        out.m_Syy.assign(in.m_Syy.begin(), in.m_Syy.end());
        out.m_Syz.assign(in.m_Syz.begin(), in.m_Syz.end());
        out.m_Szz.assign(in.m_Szz.begin(), in.m_Szz.end());

        out.m_timestamp_sec = in.m_timestamp_sec;
        out.m_frame_id = in.m_frame_id;
    }

    void NormalizedPointClouds::centerPointCloud(
        const PointCloudView& in, 
        PointCloud& out,
        std::array<float, 3>& mean
    )
    {
        for(std::size_t idx {0}; idx < in.m_x.size(); ++idx)
        {
            mean[0] += in.m_x[idx];
            mean[1] += in.m_y[idx];
            mean[2] += in.m_z[idx];
        }
        mean[0] /= in.m_x.size();
        mean[1] /= in.m_y.size();
        mean[2] /= in.m_z.size();

        for(std::size_t idx {0}; idx < in.m_x.size(); ++idx)
        {
            out.m_x[idx] = in.m_x[idx] - mean[0];
            out.m_y[idx] = in.m_y[idx] - mean[1];
            out.m_z[idx] = in.m_z[idx] - mean[2];
        }  
    }

    float NormalizedPointClouds::computeScale(const PointCloudView& in)
    {
        float scale {0.0f};
        for(std::size_t idx {0}; idx < in.m_x.size(); ++idx)
        {
            scale += (
                in.m_x[idx]*in.m_x[idx]
                + in.m_y[idx]*in.m_y[idx]
                + in.m_z[idx]*in.m_z[idx]
            );
        }
        scale = std::sqrt(scale/in.m_x.size());
        return scale;
    }

    void NormalizedPointClouds::scalePointCloud(
        PointCloud& X
    )
    {
        float scale_sq = m_info.m_scale*m_info.m_scale;
        for(std::size_t idx {0}; idx < X.m_x.size(); ++idx)
        {
            X.m_x[idx] /= m_info.m_scale;
            X.m_y[idx] /= m_info.m_scale;
            X.m_z[idx] /= m_info.m_scale;
            X.m_Sxx[idx] /= scale_sq;
            X.m_Sxy[idx] /= scale_sq;
            X.m_Sxz[idx] /= scale_sq;
            X.m_Syy[idx] /= scale_sq;
            X.m_Syz[idx] /= scale_sq;
            X.m_Szz[idx] /= scale_sq;
        }       
    }

    void NormalizedPointClouds::normalizePointClouds(
        const PointCloudView& in_X,
        const PointCloudView& in_Y
    )
    {   
        m_info = NormalizationInfo{};
        
        allocatePointCloud(in_X, m_X);
        allocatePointCloud(in_Y, m_Y);

        centerPointCloud(in_X, m_X, m_info.m_X_mean);
        centerPointCloud(in_Y, m_Y, m_info.m_Y_mean);

        float scale_X = computeScale(makePointCloudView(m_X));
        float scale_Y = computeScale(makePointCloudView(m_Y));
        
        m_info.m_scale = (scale_X > scale_Y) ? scale_X : scale_Y;

        // to be removed later: begin
        std::fprintf(
            stderr,
            "[bot_dugma::normalize] scale_X=%.9e scale_Y=%.9e chosen_scale=%.9e\n",
            static_cast<double>(scale_X),
            static_cast<double>(scale_Y),
            static_cast<double>(m_info.m_scale)
        );
        // to be removed later: end
        
        scalePointCloud(m_X);
        scalePointCloud(m_Y);
    }

    PoseEstimate denormalizePose(
        const PoseEstimate& normalized_pose,
        const NormalizationInfo& normalization_info
    )
    {
        PoseEstimate out = normalized_pose;

        const float t_norm_x = normalized_pose.m_t[0];
        const float t_norm_y = normalized_pose.m_t[1];
        const float t_norm_z = normalized_pose.m_t[2];

        const float Ry_mean_x =
            normalized_pose.m_R[0] * normalization_info.m_Y_mean[0] +
            normalized_pose.m_R[1] * normalization_info.m_Y_mean[1] +
            normalized_pose.m_R[2] * normalization_info.m_Y_mean[2];

        const float Ry_mean_y =
            normalized_pose.m_R[3] * normalization_info.m_Y_mean[0] +
            normalized_pose.m_R[4] * normalization_info.m_Y_mean[1] +
            normalized_pose.m_R[5] * normalization_info.m_Y_mean[2];

        const float Ry_mean_z =
            normalized_pose.m_R[6] * normalization_info.m_Y_mean[0] +
            normalized_pose.m_R[7] * normalization_info.m_Y_mean[1] +
            normalized_pose.m_R[8] * normalization_info.m_Y_mean[2];

        out.m_t[0] =
            normalization_info.m_X_mean[0]
            - Ry_mean_x
            + normalization_info.m_scale * t_norm_x;

        out.m_t[1] =
            normalization_info.m_X_mean[1]
            - Ry_mean_y
            + normalization_info.m_scale * t_norm_y;

        out.m_t[2] =
            normalization_info.m_X_mean[2]
            - Ry_mean_z
            + normalization_info.m_scale * t_norm_z;

        return out;
    }

    PoseEstimate normalizePose(
        const PoseEstimate& original_pose,
        const NormalizationInfo& normalization_info
    )
    {
        PoseEstimate out = original_pose;

        const float Ry_mean_x =
            original_pose.m_R[0] * normalization_info.m_Y_mean[0] +
            original_pose.m_R[1] * normalization_info.m_Y_mean[1] +
            original_pose.m_R[2] * normalization_info.m_Y_mean[2];

        const float Ry_mean_y =
            original_pose.m_R[3] * normalization_info.m_Y_mean[0] +
            original_pose.m_R[4] * normalization_info.m_Y_mean[1] +
            original_pose.m_R[5] * normalization_info.m_Y_mean[2];

        const float Ry_mean_z =
            original_pose.m_R[6] * normalization_info.m_Y_mean[0] +
            original_pose.m_R[7] * normalization_info.m_Y_mean[1] +
            original_pose.m_R[8] * normalization_info.m_Y_mean[2];

        out.m_t[0] =
            (Ry_mean_x + original_pose.m_t[0] - normalization_info.m_X_mean[0]) /
            normalization_info.m_scale;

        out.m_t[1] =
            (Ry_mean_y + original_pose.m_t[1] - normalization_info.m_X_mean[1]) /
            normalization_info.m_scale;

        out.m_t[2] =
            (Ry_mean_z + original_pose.m_t[2] - normalization_info.m_X_mean[2]) /
            normalization_info.m_scale;

        return out;
    }
}
