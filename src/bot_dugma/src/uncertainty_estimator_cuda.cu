#include "uncertainty_estimator_cuda.hpp"
#include "cuda_utils.hpp"

#include <cuda_runtime.h>

namespace bot_dugma
{
    __constant__ float d_mle_R[9];
    __constant__ float d_mle_t[3];
    __constant__ float d_mle_R_coeffs[36];
    constexpr float kWeightEps = 1.0e-12f;
    constexpr float kMinDetS = 1.0e-12f;
    constexpr int kPackedHSize = 21;
    constexpr int kInfoBlockX = 16;
    constexpr int kInfoBlockY = 16;
    constexpr int kInfoBlockThreads = kInfoBlockX * kInfoBlockY;

    void copyToDevicePose(const PoseEstimate& pose)
    {
        checkCuda(
            cudaMemcpyToSymbol(d_mle_R, pose.m_R, 9 * sizeof(float)),
            "copy mle R to constant memory"
        );
        checkCuda(
            cudaMemcpyToSymbol(d_mle_t, pose.m_t, 3 * sizeof(float)),
            "copy mle t to constant memory"
        );

        float R_coeffs[36] = {
            // Ryxx
            pose.m_R[0]*pose.m_R[0],
            2.0f*pose.m_R[0]*pose.m_R[1],
            2.0f*pose.m_R[0]*pose.m_R[2],
            pose.m_R[1]*pose.m_R[1],
            2.0f*pose.m_R[1]*pose.m_R[2],
            pose.m_R[2]*pose.m_R[2],

            // Ryxy
            pose.m_R[3]*pose.m_R[0],
            pose.m_R[3]*pose.m_R[1] + pose.m_R[4]*pose.m_R[0],
            pose.m_R[3]*pose.m_R[2] + pose.m_R[5]*pose.m_R[0],
            pose.m_R[4]*pose.m_R[1],
            pose.m_R[4]*pose.m_R[2] + pose.m_R[5]*pose.m_R[1],
            pose.m_R[5]*pose.m_R[2],

            // Ryxz
            pose.m_R[6]*pose.m_R[0],
            pose.m_R[6]*pose.m_R[1] + pose.m_R[7]*pose.m_R[0],
            pose.m_R[6]*pose.m_R[2] + pose.m_R[8]*pose.m_R[0],
            pose.m_R[7]*pose.m_R[1],
            pose.m_R[7]*pose.m_R[2] + pose.m_R[8]*pose.m_R[1],
            pose.m_R[8]*pose.m_R[2],

            // Ryyy
            pose.m_R[3]*pose.m_R[3],
            2.0f*pose.m_R[3]*pose.m_R[4],
            2.0f*pose.m_R[3]*pose.m_R[5],
            pose.m_R[4]*pose.m_R[4],
            2.0f*pose.m_R[4]*pose.m_R[5],
            pose.m_R[5]*pose.m_R[5],

            // Ryyz
            pose.m_R[6]*pose.m_R[3],
            pose.m_R[6]*pose.m_R[4] + pose.m_R[7]*pose.m_R[3],
            pose.m_R[6]*pose.m_R[5] + pose.m_R[8]*pose.m_R[3],
            pose.m_R[7]*pose.m_R[4],
            pose.m_R[7]*pose.m_R[5] + pose.m_R[8]*pose.m_R[4],
            pose.m_R[8]*pose.m_R[5],

            // Ryzz
            pose.m_R[6]*pose.m_R[6],
            2.0f*pose.m_R[6]*pose.m_R[7],
            2.0f*pose.m_R[6]*pose.m_R[8],
            pose.m_R[7]*pose.m_R[7],
            2.0f*pose.m_R[7]*pose.m_R[8],
            pose.m_R[8]*pose.m_R[8]
        };

        checkCuda(
            cudaMemcpyToSymbol(d_mle_R_coeffs, R_coeffs, 36 * sizeof(float)),
            "copy mle R coeffs to constant memory"
        );
    }

    __global__ void computeColumnSumsKernel(
        const float* C_old,
        std::size_t N,
        std::size_t M,
        float* col_sums
    )
    {
        const int j = blockIdx.x * blockDim.x + threadIdx.x;
        if(j >= static_cast<int>(M))
        {
            return;
        }

        float sum = 0.0f;
        for(std::size_t i = 0; i < N; ++i)
        {
            sum += C_old[j * N + i];
        }
        col_sums[j] = sum;
    }

    __global__ void computeInfoMatrixPartialsKernel(
        DevicePointCloud X0,
        DevicePointCloud Y0,
        const float* C_old,
        const float* col_sums,
        float* H_partial
    )
    {
        __shared__ float H_shared[kInfoBlockThreads * kPackedHSize];

        const int tid = threadIdx.y * blockDim.x + threadIdx.x;
        const int idx = blockDim.x * blockIdx.x + threadIdx.x;
        const int idy = blockDim.y * blockIdx.y + threadIdx.y;

        #pragma unroll
        for(int k = 0; k < kPackedHSize; ++k)
        {
            H_shared[tid * kPackedHSize + k] = 0.0f;
        }

        if(idx < static_cast<int>(X0.count) && idy < static_cast<int>(Y0.count))
        {
            const float Cij = C_old[idy * X0.count + idx];
            const float wij = Cij / (col_sums[idy] + kWeightEps);

            const float Xsxx = X0.Sxx[idx];
            const float Xsxy = X0.Sxy[idx];
            const float Xsxz = X0.Sxz[idx];
            const float Xsyy = X0.Syy[idx];
            const float Xsyz = X0.Syz[idx];
            const float Xszz = X0.Szz[idx];

            const float Y0x = Y0.x[idy];
            const float Y0y = Y0.y[idy];
            const float Y0z = Y0.z[idy];

            const float Ysxx = Y0.Sxx[idy];
            const float Ysxy = Y0.Sxy[idy];
            const float Ysxz = Y0.Sxz[idy];
            const float Ysyy = Y0.Syy[idy];
            const float Ysyz = Y0.Syz[idy];
            const float Yszz = Y0.Szz[idy];

            const float Ryxx =
                d_mle_R_coeffs[0]*Ysxx + d_mle_R_coeffs[1]*Ysxy + d_mle_R_coeffs[2]*Ysxz
                + d_mle_R_coeffs[3]*Ysyy + d_mle_R_coeffs[4]*Ysyz + d_mle_R_coeffs[5]*Yszz;

            const float Ryxy =
                d_mle_R_coeffs[6]*Ysxx + d_mle_R_coeffs[7]*Ysxy + d_mle_R_coeffs[8]*Ysxz
                + d_mle_R_coeffs[9]*Ysyy + d_mle_R_coeffs[10]*Ysyz + d_mle_R_coeffs[11]*Yszz;

            const float Ryxz =
                d_mle_R_coeffs[12]*Ysxx + d_mle_R_coeffs[13]*Ysxy + d_mle_R_coeffs[14]*Ysxz
                + d_mle_R_coeffs[15]*Ysyy + d_mle_R_coeffs[16]*Ysyz + d_mle_R_coeffs[17]*Yszz;

            const float Ryyy =
                d_mle_R_coeffs[18]*Ysxx + d_mle_R_coeffs[19]*Ysxy + d_mle_R_coeffs[20]*Ysxz
                + d_mle_R_coeffs[21]*Ysyy + d_mle_R_coeffs[22]*Ysyz + d_mle_R_coeffs[23]*Yszz;

            const float Ryyz =
                d_mle_R_coeffs[24]*Ysxx + d_mle_R_coeffs[25]*Ysxy + d_mle_R_coeffs[26]*Ysxz
                + d_mle_R_coeffs[27]*Ysyy + d_mle_R_coeffs[28]*Ysyz + d_mle_R_coeffs[29]*Yszz;

            const float Ryzz =
                d_mle_R_coeffs[30]*Ysxx + d_mle_R_coeffs[31]*Ysxy + d_mle_R_coeffs[32]*Ysxz
                + d_mle_R_coeffs[33]*Ysyy + d_mle_R_coeffs[34]*Ysyz + d_mle_R_coeffs[35]*Yszz;

            const float Sxx = Xsxx + Ryxx;
            const float Sxy = Xsxy + Ryxy;
            const float Sxz = Xsxz + Ryxz;
            const float Syy = Xsyy + Ryyy;
            const float Syz = Xsyz + Ryyz;
            const float Szz = Xszz + Ryzz;

            const float detS =
                  Sxx*(Syy*Szz - Syz*Syz)
                - Sxy*(Sxy*Szz - Syz*Sxz)
                + Sxz*(Sxy*Syz - Syy*Sxz);

            if(isfinite(detS) && detS > kMinDetS)
            {
                const float Lxx = (Syy*Szz - Syz*Syz) / detS;
                const float Lxy = -(Sxy*Szz - Syz*Sxz) / detS;
                const float Lxz = (Sxy*Syz - Syy*Sxz) / detS;
                const float Lyy = (Sxx*Szz - Sxz*Sxz) / detS;
                const float Lyz = -(Sxx*Syz - Sxy*Sxz) / detS;
                const float Lzz = (Sxx*Syy - Sxy*Sxy) / detS;

                const float Yx =
                    d_mle_R[0]*Y0x + d_mle_R[1]*Y0y + d_mle_R[2]*Y0z + d_mle_t[0];

                const float Yy =
                    d_mle_R[3]*Y0x + d_mle_R[4]*Y0y + d_mle_R[5]*Y0z + d_mle_t[1];

                const float Yz =
                    d_mle_R[6]*Y0x + d_mle_R[7]*Y0y + d_mle_R[8]*Y0z + d_mle_t[2];

                H_shared[tid * kPackedHSize + 0]  = wij * Lxx;
                H_shared[tid * kPackedHSize + 1]  = wij * Lxy;
                H_shared[tid * kPackedHSize + 2]  = wij * Lxz;
                H_shared[tid * kPackedHSize + 3]  = wij * (-Lxy*Yz + Lxz*Yy);
                H_shared[tid * kPackedHSize + 4]  = wij * ( Lxx*Yz - Lxz*Yx);
                H_shared[tid * kPackedHSize + 5]  = wij * (-Lxx*Yy + Lxy*Yx);

                H_shared[tid * kPackedHSize + 6]  = wij * Lyy;
                H_shared[tid * kPackedHSize + 7]  = wij * Lyz;
                H_shared[tid * kPackedHSize + 8]  = wij * (-Lyy*Yz + Lyz*Yy);
                H_shared[tid * kPackedHSize + 9]  = wij * ( Lxy*Yz - Lyz*Yx);
                H_shared[tid * kPackedHSize + 10] = wij * (-Lxy*Yy + Lyy*Yx);

                H_shared[tid * kPackedHSize + 11] = wij * Lzz;
                H_shared[tid * kPackedHSize + 12] = wij * (-Lyz*Yz + Lzz*Yy);
                H_shared[tid * kPackedHSize + 13] = wij * ( Lxz*Yz - Lzz*Yx);
                H_shared[tid * kPackedHSize + 14] = wij * (-Lxz*Yy + Lyz*Yx);

                H_shared[tid * kPackedHSize + 15] =
                    wij * (Lyy*Yz*Yz - 2.0f*Lyz*Yy*Yz + Lzz*Yy*Yy);
                H_shared[tid * kPackedHSize + 16] =
                    wij * (-Lxy*Yz*Yz + Lxz*Yy*Yz + Lyz*Yx*Yz - Lzz*Yx*Yy);
                H_shared[tid * kPackedHSize + 17] =
                    wij * ( Lxy*Yy*Yz - Lyy*Yx*Yz - Lxz*Yy*Yy + Lyz*Yx*Yy);

                H_shared[tid * kPackedHSize + 18] =
                    wij * (Lxx*Yz*Yz - 2.0f*Lxz*Yx*Yz + Lzz*Yx*Yx);
                H_shared[tid * kPackedHSize + 19] =
                    wij * (-Lxx*Yy*Yz + Lxy*Yx*Yz + Lxz*Yx*Yy - Lyz*Yx*Yx);

                H_shared[tid * kPackedHSize + 20] =
                    wij * (Lxx*Yy*Yy - 2.0f*Lxy*Yx*Yy + Lyy*Yx*Yx);
            }
        }

        __syncthreads();

        for(int stride = kInfoBlockThreads / 2; stride > 0; stride >>= 1)
        {
            if(tid < stride)
            {
                #pragma unroll
                for(int k = 0; k < kPackedHSize; ++k)
                {
                    H_shared[tid * kPackedHSize + k] +=
                        H_shared[(tid + stride) * kPackedHSize + k];
                }
            }
            __syncthreads();
        }

        if(tid == 0)
        {
            const int block_id = blockIdx.y * gridDim.x + blockIdx.x;
            #pragma unroll
            for(int k = 0; k < kPackedHSize; ++k)
            {
                H_partial[block_id * kPackedHSize + k] = H_shared[k];
            }
        }
    }

    __global__ void reduceInfoMatrixPartialsKernel(
        const float* H_partial,
        int num_blocks,
        float* H_out
    )
    {
        const int k = threadIdx.x;
        if(k >= kPackedHSize)
        {
            return;
        }

        float sum = 0.0f;
        for(int b = 0; b < num_blocks; ++b)
        {
            sum += H_partial[b * kPackedHSize + k];
        }
        H_out[k] = sum;
    }

    void launchComputeInfoMatrixKernel(
        const DeviceRegistrationData& device_data,
        const PoseEstimate& pose,
        std::array<float, 21>& H_out_h
    )
    {
        copyToDevicePose(pose);

        float* col_sums = nullptr;
        float* H_partial = nullptr;
        float* H_out_d = nullptr;
        allocateOnDevice(
            col_sums,
            device_data.Y0().count,
            "cudaMalloc col_sums"
        );

        constexpr int t_per_blk_1d = 256;
        const int blk_count_1d = static_cast<int>(
            (device_data.Y0().count + t_per_blk_1d - 1) / t_per_blk_1d
        );

        computeColumnSumsKernel<<<blk_count_1d, t_per_blk_1d>>>(
            device_data.COld(),
            device_data.X0().count,
            device_data.Y0().count,
            col_sums
        );
        checkCuda(cudaGetLastError(), "computeColumnSumsKernel launch");
        checkCuda(cudaDeviceSynchronize(), "computeColumnSumsKernel sync");

        constexpr int bx {kInfoBlockX}, by {kInfoBlockY};
        dim3 dim_block(bx, by, 1);
        dim3 dim_grid(
            static_cast<int>((device_data.X0().count + bx - 1)/bx),
            static_cast<int>((device_data.Y0().count + by - 1)/by),
            1
        );
        const int num_blocks = static_cast<int>(dim_grid.x * dim_grid.y);

        allocateOnDevice(
            H_partial,
            static_cast<std::size_t>(num_blocks) * kPackedHSize,
            "cudaMalloc H_partial"
        );
        allocateOnDevice(
            H_out_d,
            kPackedHSize,
            "cudaMalloc H_out_d"
        );

        computeInfoMatrixPartialsKernel<<<dim_grid, dim_block>>>(
            device_data.X0(),
            device_data.Y0(),
            device_data.COld(),
            col_sums,
            H_partial
        );
        checkCuda(cudaGetLastError(), "computeInfoMatrixPartialsKernel launch");
        checkCuda(cudaDeviceSynchronize(), "computeInfoMatrixPartialsKernel sync");

        reduceInfoMatrixPartialsKernel<<<1, 32>>>(
            H_partial,
            num_blocks,
            H_out_d
        );
        checkCuda(cudaGetLastError(), "reduceInfoMatrixPartialsKernel launch");
        checkCuda(cudaDeviceSynchronize(), "reduceInfoMatrixPartialsKernel sync");

        copyToHost(
            H_out_h.data(),
            H_out_d,
            kPackedHSize,
            "copy H_out_d to host"
        );

        freeOnDevice(col_sums);
        freeOnDevice(H_partial);
        freeOnDevice(H_out_d);
    }
}
