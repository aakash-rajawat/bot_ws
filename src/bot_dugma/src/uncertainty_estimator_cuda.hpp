#ifndef UNCERTAINTY_ESTIMATOR_CUDA_HPP
#define UNCERTAINTY_ESTIMATOR_CUDA_HPP

#include "bot_dugma/uncertainty_estimator.hpp"

namespace bot_dugma
{
    void copyToDevicePose(const PoseEstimate& pose);

    void launchComputeInfoMatrixKernel(
        const DeviceRegistrationData& device_data,
        const PoseEstimate& pose,
        std::array<float, 21>& H_out_h
    );
}


#endif
