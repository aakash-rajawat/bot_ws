#ifndef E_STEP_CUDA_HPP
#define E_STEP_CUDA_HPP

#include "bot_dugma/e_step.hpp"

namespace bot_dugma
{
    void copyToDevicePoseCuda(const PoseEstimate& pose);

    void launchMovingCloudTransformKernel(
        DeviceRegistrationData& device_data
    );

    float launchComputeMinDistKernel(
        const DeviceRegistrationData& device_data
    );

    void launchUpdateCovariancesKernel(
        DeviceRegistrationData& device_data,
        float mean_min_dists
    );

    void launchComputeCOldCoeffsKernel(
        DeviceRegistrationData& device_data
    );    

    BasisCoeffsVec launchComputeACoeffsKernel(
        DeviceRegistrationData& device_data
    );
}

#endif