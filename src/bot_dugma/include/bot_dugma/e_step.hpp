#ifndef E_STEP_HPP
#define E_STEP_HPP

#include "bot_dugma/initialize.hpp"
#include "bot_dugma/types.hpp"

#include <array>

namespace bot_dugma
{
    struct BasisCoeffsVec
    {
        static constexpr int kSize = 637;
        std::array<float, kSize> vals {};
    };

    class EStep
    {
    public:
        explicit EStep(
            DeviceRegistrationData& device_data,
            const PoseEstimate& pose
        );

        EStep(const EStep&) = delete;
        EStep& operator=(const EStep&) = delete;

        const BasisCoeffsVec& basisCoeffs() const noexcept;

    private:
        void copyToDevicePose(const PoseEstimate& pose);

        BasisCoeffsVec runEStep(
            DeviceRegistrationData& device_data
        );

        void transformMovingCloud(
            DeviceRegistrationData& device_data
        );

        float computeMinDist(
            const DeviceRegistrationData& device_data
        );

        void updateCovariances(
            DeviceRegistrationData& device_data,
            float mean_min_dists
        );

        void computeCOldCoeffs(
            DeviceRegistrationData& device_data
        );

        BasisCoeffsVec computeACoeffs(
            DeviceRegistrationData& device_data
        );

        BasisCoeffsVec m_A_h {};
    };
}


#endif