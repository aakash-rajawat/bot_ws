#include "initialize_cuda.hpp"
#include "cuda_utils.hpp"

#include <cmath>
#include <cuda_runtime.h>

namespace bot_dugma
{
    __global__ void precomputeCovarianceTermsKernel(DevicePointCloud pt_cloud)
    {
        // each thread corresponds to a point
        const std::size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= pt_cloud.count)
        {
            return;
        }

        // instead of doing global memory calls again and again later
        // call them on the register
        const float Sxx = pt_cloud.Sxx[idx];
        const float Sxy = pt_cloud.Sxy[idx];
        const float Sxz = pt_cloud.Sxz[idx];
        const float Syy = pt_cloud.Syy[idx];
        const float Syz = pt_cloud.Syz[idx];
        const float Szz = pt_cloud.Szz[idx];
        
        // determinant(covariance) computation
        const float det =
              Sxx*(Syy*Szz - Syz*Syz)
            - Sxy*(Sxy*Szz - Syz*Sxz)
            + Sxz*(Sxy*Syz - Syy*Sxz);

        if(!(isfinite(det) && det > 0.0f))
        {
            pt_cloud.invSqrtDet[idx] = 0.0f;

            pt_cloud.iSxx[idx] = 0.0f;
            pt_cloud.iSxy[idx] = 0.0f;
            pt_cloud.iSxz[idx] = 0.0f;
            pt_cloud.iSyy[idx] = 0.0f;
            pt_cloud.iSyz[idx] = 0.0f;
            pt_cloud.iSzz[idx] = 0.0f;
            return;
        }

        // 1/square_root(det) computation
        pt_cloud.invSqrtDet[idx] = rsqrtf(det);

        // inverse(covariance) computation
        pt_cloud.iSxx[idx] = (Syy*Szz - Syz*Syz)/det;
        pt_cloud.iSxy[idx] = -(Sxy*Szz - Syz*Sxz)/det;
        pt_cloud.iSxz[idx] = (Sxy*Syz - Syy*Sxz)/det;
        pt_cloud.iSyy[idx] = (Sxx*Szz - Sxz*Sxz)/det;
        pt_cloud.iSyz[idx] = -(Sxx*Syz - Sxy*Sxz)/det;
        pt_cloud.iSzz[idx] = (Sxx*Syy - Sxy*Sxy)/det;
    }

    void launchPrecomputeCovarianceTermsKernel(
    DevicePointCloud& pt_cloud,
    int blk_count,
    int t_per_blk
    )
    {
        precomputeCovarianceTermsKernel<<<blk_count, t_per_blk>>>(pt_cloud);

        checkCuda(cudaGetLastError(), "precomputeCovarianceTermsKernel launch");
        checkCuda(cudaDeviceSynchronize(), "precomputeCovarianceTermsKernel sync");
    }
}
