#include "bot_dugma/e_step.hpp"
#include "e_step_cuda.hpp"

namespace bot_dugma
{
    EStep::EStep(DeviceRegistrationData& device_data, const PoseEstimate& pose)
    {
        copyToDevicePose(pose);
        m_A_h = runEStep(device_data);
    }

    void EStep::copyToDevicePose(const PoseEstimate& pose)
    {
        copyToDevicePoseCuda(pose);
    }

    BasisCoeffsVec EStep::runEStep(
        DeviceRegistrationData& device_data
    )
    {
        transformMovingCloud(device_data);

        const float mean_min_dists = computeMinDist(device_data);

        updateCovariances(device_data, mean_min_dists);

        computeCOldCoeffs(device_data);

        return computeACoeffs(device_data);
    }

    void EStep::transformMovingCloud(
        DeviceRegistrationData& device_data
    )
    {
        launchMovingCloudTransformKernel(device_data);
    }

    float EStep::computeMinDist(const DeviceRegistrationData& device_data)
    {
        return launchComputeMinDistKernel(device_data);
    }

    void EStep::updateCovariances(
        DeviceRegistrationData& device_data,
        float mean_min_dists
    )
    {
        if(mean_min_dists <= 0.0f)
        {
            return;
        }
        launchUpdateCovariancesKernel(device_data, mean_min_dists);
    }

    void EStep::computeCOldCoeffs(
        DeviceRegistrationData& device_data
    )
    {
        launchComputeCOldCoeffsKernel(device_data);
    }

    BasisCoeffsVec EStep::computeACoeffs(
        DeviceRegistrationData& device_data
    )
    {
        BasisCoeffsVec A = launchComputeACoeffsKernel(device_data);
        return A;
    }

    const BasisCoeffsVec& EStep::basisCoeffs() const noexcept
    {
        return m_A_h;
    }
}
