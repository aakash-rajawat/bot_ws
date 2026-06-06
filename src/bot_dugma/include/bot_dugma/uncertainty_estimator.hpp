#ifndef UNCERTAINTY_ESTIMATOR_HPP
#define UNCERTAINTY_ESTIMATOR_HPP

#include "bot_dugma/types.hpp"
#include "bot_dugma/initialize.hpp"
#include "bot_dugma/normalize.hpp"

#include <array>

namespace bot_dugma
{
    class UncertaintyEstimator
    {
    public:
        UncertaintyEstimator(
            const DeviceRegistrationData& device_data,
            const PoseEstimate& pose,
            const NormalizationInfo& norm_info
        );

        bool hasValidCovariance() const noexcept;
        const std::array<float, 21>& normalizedPoseCovariance() const noexcept;
        const std::array<float, 21>& denormalizedPoseCovariance() const noexcept;

    private:
        void computeInfoMatrix();

        void computeNormalizedPoseCov();

        std::array<float, 21> computeDenormalizedPoseCov();

        const DeviceRegistrationData* m_device_data {nullptr};
        PoseEstimate m_pose {};
        NormalizationInfo m_norm_info {};
        std::array<float, 21> m_H_n {};
        std::array<float, 21> m_cov_n {};
        std::array<float, 21> m_cov_orig {};
        bool m_has_valid_covariance {false};

    };
}


#endif
