#include "bot_dugma/uncertainty_estimator.hpp"
#include "uncertainty_estimator_cuda.hpp"

#include <Eigen/Dense>

namespace bot_dugma
{
    namespace
    {
        Eigen::Matrix<float, 6, 6> unpackSymmetric6(
            const std::array<float, 21>& packed
        )
        {
            Eigen::Matrix<float, 6, 6> M = Eigen::Matrix<float, 6, 6>::Zero();

            int k = 0;
            for(int i = 0; i < 6; ++i)
            {
                for(int j = i; j < 6; ++j)
                {
                    M(i,j) = packed[k];
                    M(j,i) = packed[k];
                    ++k;
                }
            }

            return M;
        }

        std::array<float, 21> packSymmetric6(
            const Eigen::Matrix<float, 6, 6>& M
        )
        {
            std::array<float, 21> packed {};
            int k = 0;
            for(int i = 0; i < 6; ++i)
            {
                for(int j = i; j < 6; ++j)
                {
                    packed[k] = M(i,j);
                    ++k;
                }
            }
            return packed;
        }

        Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& v)
        {
            Eigen::Matrix3f S;
            S <<
                 0.0f, -v.z(),  v.y(),
                 v.z(),  0.0f, -v.x(),
                -v.y(),  v.x(),  0.0f;
            return S;
        }
    }

    UncertaintyEstimator::UncertaintyEstimator(
        const DeviceRegistrationData& device_data,
        const PoseEstimate& pose,
        const NormalizationInfo& norm_info
    )
    {
        m_device_data = &device_data;
        m_pose = pose;
        m_norm_info = norm_info;

        computeInfoMatrix();
        computeNormalizedPoseCov();
        if(m_has_valid_covariance)
        {
            m_cov_orig = computeDenormalizedPoseCov();
        }
        else
        {
            m_cov_orig.fill(0.0f);
        }
    }

    void UncertaintyEstimator::computeInfoMatrix()
    {
        launchComputeInfoMatrixKernel(
            *m_device_data,
            m_pose,
            m_H_n
        );
    }

    void UncertaintyEstimator::computeNormalizedPoseCov()
    {
        m_has_valid_covariance = false;

        Eigen::Matrix<float, 6, 6> H = unpackSymmetric6(m_H_n);

        constexpr float kLambda = 1.0e-6f;
        H.diagonal().array() += kLambda;

        Eigen::FullPivLU<Eigen::Matrix<float, 6, 6>> lu(H);
        if(!lu.isInvertible())
        {
            m_cov_n.fill(0.0f);
            return;
        }

        const Eigen::Matrix<float, 6, 6> cov = lu.inverse();
        if(!cov.allFinite())
        {
            m_cov_n.fill(0.0f);
            return;
        }

        m_cov_n = packSymmetric6(cov);
        m_has_valid_covariance = true;
    }

    std::array<float, 21> UncertaintyEstimator::computeDenormalizedPoseCov()
    {
        const Eigen::Matrix<float, 6, 6> cov_n = unpackSymmetric6(m_cov_n);

        const Eigen::Vector3f mu_X(
            m_norm_info.m_X_mean[0],
            m_norm_info.m_X_mean[1],
            m_norm_info.m_X_mean[2]
        );

        Eigen::Matrix<float, 6, 6> J =
            Eigen::Matrix<float, 6, 6>::Identity();

        J.topLeftCorner<3, 3>() *= m_norm_info.m_scale;
        J.topRightCorner<3, 3>() = skewSymmetric(mu_X);

        const Eigen::Matrix<float, 6, 6> cov_orig =
            J * cov_n * J.transpose();

        return packSymmetric6(cov_orig);
    }

    bool UncertaintyEstimator::hasValidCovariance() const noexcept
    {
        return m_has_valid_covariance;
    }

    const std::array<float, 21>&
    UncertaintyEstimator::normalizedPoseCovariance() const noexcept
    {
        return m_cov_n;
    }

    const std::array<float, 21>&
    UncertaintyEstimator::denormalizedPoseCovariance() const noexcept
    {
        return m_cov_orig;
    }
}
