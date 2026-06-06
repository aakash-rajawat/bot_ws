#ifndef INITIALIZE_CUDA_HPP
#define INITIALIZE_CUDA_HPP

#include "bot_dugma/initialize.hpp"
#include "cuda_utils.hpp"


namespace bot_dugma
{
    void launchPrecomputeCovarianceTermsKernel(
        DevicePointCloud& pt_cloud,
        int blk_count,
        int t_per_blk
    );
}

#endif
