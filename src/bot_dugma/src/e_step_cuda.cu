#include "e_step_cuda.hpp"
#include "cuda_utils.hpp"


#include <cmath>
#include <cfloat>
#include <cstdio>
#include <array>
#include <vector>
#include <cuda_runtime.h>

namespace bot_dugma
{
    __constant__ float d_R[9];
    __constant__ float d_t[3];
    __constant__ float d_R_coeffs[36];

    constexpr float pi = 3.14159265358979323846f;
    constexpr float pi_factor = 1.0f/(8.0f*pi*pi*pi);
    constexpr float kMaxInvSqrtDet = 1.0e8f;

    __device__ float clampFinitePositive(float value, float max_value)
    {
        if(!isfinite(value) || value < 0.0f)
        {
            return 0.0f;
        }

        return fminf(value, max_value);
    }


    namespace
    {
        constexpr int kMaxEStepSummaries {20};
        int g_e_step_summaries_printed {0};

        void printArraySummary(
            const char* name,
            const float* device_values,
            std::size_t count
        )
        {
            if(g_e_step_summaries_printed >= kMaxEStepSummaries || count == 0)
            {
                return;
            }

            std::vector<float> values {};
            copyToHost(values, device_values, count, "copy e-step summary values");

            std::size_t finite_count {0};
            std::size_t nonfinite_count {0};
            std::size_t zero_count {0};
            float min_value {FLT_MAX};
            float max_value {-FLT_MAX};
            float max_abs_value {0.0f};
            std::size_t max_abs_idx {0};
            double sum_abs {0.0};

            for(std::size_t idx = 0; idx < values.size(); ++idx)
            {
                const float value = values[idx];
                if(!std::isfinite(value))
                {
                    ++nonfinite_count;
                    continue;
                }

                ++finite_count;
                zero_count += value == 0.0f ? 1 : 0;
                min_value = fminf(min_value, value);
                max_value = fmaxf(max_value, value);

                const float abs_value = fabsf(value);
                sum_abs += static_cast<double>(abs_value);
                if(abs_value > max_abs_value)
                {
                    max_abs_value = abs_value;
                    max_abs_idx = idx;
                }
            }

            const double mean_abs =
                finite_count > 0 ?
                sum_abs / static_cast<double>(finite_count) :
                0.0;

            std::fprintf(
                stderr,
                "[bot_dugma::e_step] %s summary finite=%zu nonfinite=%zu zeros=%zu "
                "min=%.9e max=%.9e max_abs=%.9e max_abs_idx=%zu mean_abs=%.9e\n",
                name,
                finite_count,
                nonfinite_count,
                zero_count,
                static_cast<double>(finite_count > 0 ? min_value : 0.0f),
                static_cast<double>(finite_count > 0 ? max_value : 0.0f),
                static_cast<double>(max_abs_value),
                max_abs_idx,
                mean_abs
            );
        }

        void printUpdatedCovarianceSummaries(
            const DeviceRegistrationData& device_data
        )
        {
            if(g_e_step_summaries_printed >= kMaxEStepSummaries)
            {
                return;
            }

            ++g_e_step_summaries_printed;

            printArraySummary("X_curr.iSxx", device_data.XCurr().iSxx, device_data.XCurr().count);
            printArraySummary("X_curr.invSqrtDet", device_data.XCurr().invSqrtDet, device_data.XCurr().count);
            printArraySummary("Y_curr.iSxx", device_data.YCurr().iSxx, device_data.YCurr().count);
            printArraySummary("Y_curr.invSqrtDet", device_data.YCurr().invSqrtDet, device_data.YCurr().count);
        }
    }

    
    /// @brief Copy the previous iteration's pose estimate to device
    /// @param pose pose to be copied
    void copyToDevicePoseCuda(const PoseEstimate& pose)
    {
        checkCuda(
            cudaMemcpyToSymbol(d_R, pose.m_R, 9*sizeof(float)),
            "copy R to constant memory"
        );
        checkCuda(
            cudaMemcpyToSymbol(d_t, pose.m_t, 3*sizeof(float)),
            "copy t to constant memory"
        );

        // d_R_coeffs stores factors to avoid per-point computations
        // in updateYCovariancesKernel
        float R_coeffs[36] = {
            // axx
            pose.m_R[0]*pose.m_R[0],
            2.0f*pose.m_R[0]*pose.m_R[1],
            2.0f*pose.m_R[0]*pose.m_R[2],
            pose.m_R[1]*pose.m_R[1],
            2.0f*pose.m_R[1]*pose.m_R[2],
            pose.m_R[2]*pose.m_R[2],

            // axy
            pose.m_R[0]*pose.m_R[3],
            pose.m_R[0]*pose.m_R[4] + pose.m_R[1]*pose.m_R[3],
            pose.m_R[0]*pose.m_R[5] + pose.m_R[2]*pose.m_R[3],
            pose.m_R[1]*pose.m_R[4],
            pose.m_R[1]*pose.m_R[5] + pose.m_R[2]*pose.m_R[4],
            pose.m_R[2]*pose.m_R[5],

            // axz
            pose.m_R[0]*pose.m_R[6],
            pose.m_R[0]*pose.m_R[7] + pose.m_R[1]*pose.m_R[6],
            pose.m_R[0]*pose.m_R[8] + pose.m_R[2]*pose.m_R[6],
            pose.m_R[1]*pose.m_R[7],
            pose.m_R[1]*pose.m_R[8] + pose.m_R[2]*pose.m_R[7],
            pose.m_R[2]*pose.m_R[8],

            // ayy
            pose.m_R[3]*pose.m_R[3],
            2.0f*pose.m_R[3]*pose.m_R[4],
            2.0f*pose.m_R[3]*pose.m_R[5],
            pose.m_R[4]*pose.m_R[4],
            2.0f*pose.m_R[4]*pose.m_R[5],
            pose.m_R[5]*pose.m_R[5],

            // ayz
            pose.m_R[3]*pose.m_R[6],
            pose.m_R[3]*pose.m_R[7] + pose.m_R[4]*pose.m_R[6],
            pose.m_R[3]*pose.m_R[8] + pose.m_R[5]*pose.m_R[6],
            pose.m_R[4]*pose.m_R[7],
            pose.m_R[4]*pose.m_R[8] + pose.m_R[5]*pose.m_R[7],
            pose.m_R[5]*pose.m_R[8],

            // azz
            pose.m_R[6]*pose.m_R[6],
            2.0f*pose.m_R[6]*pose.m_R[7],
            2.0f*pose.m_R[6]*pose.m_R[8],
            pose.m_R[7]*pose.m_R[7],
            2.0f*pose.m_R[7]*pose.m_R[8],
            pose.m_R[8]*pose.m_R[8]
        };

        checkCuda(
            cudaMemcpyToSymbol(d_R_coeffs, R_coeffs, 36*sizeof(float)),
            "copy R coeffs to constant memory"
        );
    }


    /// E-STEP
    /// Step 3: Y_curr <- R_prev*Y_0 + t_prev ----------------------------------

    /// @brief Step 3: Main Kernel
    /// @param Y0 Raw Y point cloud with covariances
    /// @param Y_curr Y point cloud after R, t transformation
    /// @return 
    __global__ void movingCloudTransformKernel(
        DevicePointCloud Y0,
        DeviceBufferPointCloud Y_curr

    )
    {
        int idx = blockDim.x*blockIdx.x + threadIdx.x;
        if(idx >= Y0.count)
        {
            return;
        }

        // load the point y_m on register
        // to reduce subsequent repeated global memory calls
        const float x = Y0.x[idx];
        const float y = Y0.y[idx];
        const float z = Y0.z[idx];

        Y_curr.x[idx] = x*d_R[0] + y*d_R[1] + z*d_R[2] + d_t[0];
        Y_curr.y[idx] = x*d_R[3] + y*d_R[4] + z*d_R[5] + d_t[1];
        Y_curr.z[idx] = x*d_R[6] + y*d_R[7] + z*d_R[8] + d_t[2];
    }

    /// @brief Kernel launcher
    /// @param device_data Contains raw and buffer Point Clouds and Covariances
    void launchMovingCloudTransformKernel(
        DeviceRegistrationData& device_data
    )
    {
        constexpr int t_per_blk = 256;
        const int blk_count = static_cast<int>(
            (device_data.Y0().count + t_per_blk -1)/t_per_blk
        );

        movingCloudTransformKernel<<<blk_count, t_per_blk>>>(
            device_data.Y0(),
            device_data.YCurr()
        );

        checkCuda(cudaGetLastError(), "MovingCloudTransformKernel launch");
        checkCuda(cudaDeviceSynchronize(), "MovingCloudTransformKernel sync");
    }
    /// End of Step 3 ----------------------------------------------------------


    /// Step 4: Minimum distance compute, sigma in paper -----------------------

    /// @brief Step 4: Main Kernel
    /// @param X0 Raw X point cloud with covariances
    /// @param Y_curr Y point cloud buffer, fifferent from raw Y0 (see Step 3)
    /// @param min_dists sigma as described in paper, but different definition
    /// @return 
    __global__ void computeMinDistsKernel(
        DevicePointCloud X0,
        DeviceBufferPointCloud Y_curr,
        float* min_dists
    )
    {
        constexpr int x_batch_size = 4;
        constexpr int t_per_blk = 256;

        __shared__ float X_batch_x[x_batch_size];
        __shared__ float X_batch_y[x_batch_size];
        __shared__ float X_batch_z[x_batch_size];
        __shared__ float shared_min[x_batch_size][t_per_blk];

        const int tid = threadIdx.x;
        const int x_base = blockIdx.x * x_batch_size;

        if (tid < x_batch_size) {
            const int x_idx = x_base + tid;
            if (x_idx < X0.count) {
                X_batch_x[tid] = X0.x[x_idx];
                X_batch_y[tid] = X0.y[x_idx];
                X_batch_z[tid] = X0.z[x_idx];
            } else {
                X_batch_x[tid] = 0.0f;
                X_batch_y[tid] = 0.0f;
                X_batch_z[tid] = 0.0f;
            }
        }
        __syncthreads();

        float local_min[x_batch_size] = {
            FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX
        };

        for (int y_idx = tid; y_idx < Y_curr.count; y_idx += blockDim.x) {
            const float x = Y_curr.x[y_idx];
            const float y = Y_curr.y[y_idx];
            const float z = Y_curr.z[y_idx];

            #pragma unroll
            for (int k = 0; k < x_batch_size; ++k) {
                const int x_idx = x_base + k;
                if (x_idx < X0.count) {
                    const float dx = x - X_batch_x[k];
                    const float dy = y - X_batch_y[k];
                    const float dz = z - X_batch_z[k];
                    const float dist = dx*dx + dy*dy + dz*dz;
                    local_min[k] = fminf(local_min[k], dist);
                }
            }
        }

        #pragma unroll
        for (int k = 0; k < x_batch_size; ++k) {
            shared_min[k][tid] = local_min[k];
        }
        __syncthreads();

        for (int stride = blockDim.x / 2; stride > 0; stride >>= 1) {
            if (tid < stride) {
                #pragma unroll
                for (int k = 0; k < x_batch_size; ++k) {
                    shared_min[k][tid] = fminf(
                        shared_min[k][tid],
                        shared_min[k][tid + stride]
                    );
                }
            }
            __syncthreads();
        }

        if (tid == 0) {
            #pragma unroll
            for (int k = 0; k < x_batch_size; ++k) {
                const int x_idx = x_base + k;
                if (x_idx < X0.count) {
                    min_dists[x_idx] = shared_min[k][0];
                }
            }
        }
    }

    /// @brief Launches the computeMinDistsKernel
    /// @param device_data See initilize.hpp
    /// @return sigma as described in paper, but deviation from paper definition
    ///         Swap the roles of X and Y, and swap M with N
    ///         No reason given in original implementation
    float launchComputeMinDistKernel(const DeviceRegistrationData& device_data)
    {
        float* min_dists = nullptr;
        const std::size_t x_count = device_data.X0().count;

        allocateOnDevice(
            min_dists,
            x_count,
            "cudaMalloc min_dists"
        );

        constexpr int t_per_blk = 256;
        constexpr int x_batch_size = 4;
        dim3 dim_blk(t_per_blk);
        dim3 dim_grid(
            static_cast<unsigned int>(
                (device_data.X0().count + x_batch_size - 1)/x_batch_size)
        );

        computeMinDistsKernel<<<dim_grid, dim_blk>>>(
            device_data.X0(),
            device_data.YCurr(),
            min_dists
        );

        checkCuda(cudaGetLastError(), "computeMinDistsKernel launch");
        checkCuda(cudaDeviceSynchronize(), "computeMinDistsKernel sync");

        std::vector<float> min_dists_h {};
        copyToHost(min_dists_h, min_dists, x_count, "copy min_dists to host");

        double sum = 0.0;
        std::size_t valid_count = 0;
        for(std::size_t idx {0}; idx < min_dists_h.size(); ++idx)
        {
            const float dist_sq = min_dists_h[idx];
            if(std::isfinite(dist_sq) && dist_sq >= 0.0f)
            {
                sum += std::sqrt(static_cast<double>(dist_sq));
                ++valid_count;
            }
        }

        float mean_min_dists = 0.0f;
        if(valid_count > 0)
        {
            mean_min_dists = static_cast<float>(
                sum / static_cast<double>(valid_count)
            );
        }
        freeOnDevice(min_dists);

        constexpr float kMinMeanMinDistsEarly = 1.0e-4f;
        if(!std::isfinite(mean_min_dists) || mean_min_dists <= kMinMeanMinDistsEarly)
        {
            std::fprintf(
                stderr,
                "[bot_dugma::e_step] WARNING: mean_min_dists=%.9e is too small or non-finite; "
                "clamping early to %.9e before covariance scaling\n",
                static_cast<double>(mean_min_dists),
                static_cast<double>(kMinMeanMinDistsEarly)
            );
            mean_min_dists = kMinMeanMinDistsEarly;
        }

        // to be removed later: begin
        std::fprintf(
            stderr,
            "[bot_dugma::e_step] mean_min_dists=%.9e x_count=%zu y_count=%zu\n",
            static_cast<double>(mean_min_dists),
            static_cast<std::size_t>(device_data.X0().count),
            static_cast<std::size_t>(device_data.YCurr().count)
        );
        // to be removed later: end

        return mean_min_dists;
    }

    /// Step 5: Update Covariances of raw Point Clouds X0, Y0 -> X_curr, Y_curr

    /// @brief X is the fixed cloud, so X_curr keeps X0 covariance terms.
    /// @param X0 Raw X point cloud with covariances
    /// @param X_curr Point cloud positions and covariance terms copied from X0
    /// @return 
    __global__ void updateXCovariancesKernel(
        DevicePointCloud X0,
        DeviceBufferPointCloud X_curr
    )
    {
        int idx = blockDim.x*blockIdx.x + threadIdx.x;

        if(idx >= X_curr.count)
        {
            return;
        }

        // X is the fixed cloud. Keep the old sigma-scaled form nearby while
        // debugging, but do not apply sigma to X_curr.
        // const float inv_sigma = 1.0f / mean_min_dists;

        X_curr.x[idx] = X0.x[idx];
        X_curr.y[idx] = X0.y[idx];
        X_curr.z[idx] = X0.z[idx];

        // X_curr.iSxx[idx] = inv_sigma * X0.iSxx[idx];
        // X_curr.iSxy[idx] = inv_sigma * X0.iSxy[idx];
        // X_curr.iSxz[idx] = inv_sigma * X0.iSxz[idx];
        // X_curr.iSyy[idx] = inv_sigma * X0.iSyy[idx];
        // X_curr.iSyz[idx] = inv_sigma * X0.iSyz[idx];
        // X_curr.iSzz[idx] = inv_sigma * X0.iSzz[idx];
        //
        // const float inv_sqrt_det =
        //     X0.invSqrtDet[idx] /
        //     sqrtf(mean_min_dists * mean_min_dists * mean_min_dists);
        // X_curr.invSqrtDet[idx] = clampFinitePositive(inv_sqrt_det, kMaxInvSqrtDet);

        X_curr.iSxx[idx] = X0.iSxx[idx];
        X_curr.iSxy[idx] = X0.iSxy[idx];
        X_curr.iSxz[idx] = X0.iSxz[idx];
        X_curr.iSyy[idx] = X0.iSyy[idx];
        X_curr.iSyz[idx] = X0.iSyz[idx];
        X_curr.iSzz[idx] = X0.iSzz[idx];
        X_curr.invSqrtDet[idx] = X0.invSqrtDet[idx];
    }

    /// @brief Sigma_Y_curr <- sigma*(R*Sigma_Y0*R.T)
    ///        => inv_Sigma_Y_curr <- (1/sigma)*(R*inv_Sigma_Y0*R.T)
    /// @param Y0 Raw Y point cloud with covariances
    /// @param Y_curr Transformed Y point cloud with yet unscaled covariances
    /// @param mean_min_dists Scaling factor
    /// @return 
    __global__ void updateYCovariancesKernel(
        DevicePointCloud Y0,
        DeviceBufferPointCloud Y_curr,
        float mean_min_dists
    )
    {
        int idx = blockDim.x*blockIdx.x + threadIdx.x;

        if(idx >= Y_curr.count)
        {
            return;
        }

        // get the inverse(Sigma_Y0) elements here in register
        const float isxx = Y0.iSxx[idx];
        const float isxy = Y0.iSxy[idx];
        const float isxz = Y0.iSxz[idx];
        const float isyy = Y0.iSyy[idx];
        const float isyz = Y0.iSyz[idx];
        const float iszz = Y0.iSzz[idx];

        const float inv_sigma = 1.0f / mean_min_dists;

        // Precomputed d_R_ceoffs used here to avoid per-point rotation elements
        // inter-operations
        Y_curr.iSxx[idx] = inv_sigma*(
            d_R_coeffs[0]*isxx + d_R_coeffs[1]*isxy + d_R_coeffs[2]*isxz
            + d_R_coeffs[3]*isyy + d_R_coeffs[4]*isyz + d_R_coeffs[5]*iszz
        );

        Y_curr.iSxy[idx] = inv_sigma*(
            d_R_coeffs[6]*isxx + d_R_coeffs[7]*isxy + d_R_coeffs[8]*isxz
            + d_R_coeffs[9]*isyy + d_R_coeffs[10]*isyz + d_R_coeffs[11]*iszz
        );

        Y_curr.iSxz[idx] = inv_sigma*(
            d_R_coeffs[12]*isxx + d_R_coeffs[13]*isxy + d_R_coeffs[14]*isxz
            + d_R_coeffs[15]*isyy + d_R_coeffs[16]*isyz + d_R_coeffs[17]*iszz
        );

        Y_curr.iSyy[idx] = inv_sigma*(
            d_R_coeffs[18]*isxx + d_R_coeffs[19]*isxy + d_R_coeffs[20]*isxz
            + d_R_coeffs[21]*isyy + d_R_coeffs[22]*isyz + d_R_coeffs[23]*iszz
        );

        Y_curr.iSyz[idx] = inv_sigma*(
            d_R_coeffs[24]*isxx + d_R_coeffs[25]*isxy + d_R_coeffs[26]*isxz
            + d_R_coeffs[27]*isyy + d_R_coeffs[28]*isyz + d_R_coeffs[29]*iszz
        );

        Y_curr.iSzz[idx] = inv_sigma*(
            d_R_coeffs[30]*isxx + d_R_coeffs[31]*isxy + d_R_coeffs[32]*isxz
            + d_R_coeffs[33]*isyy + d_R_coeffs[34]*isyz + d_R_coeffs[35]*iszz
        );

        // Also update the sqrt(1/determinant(Sigma_Y_curr))
        const float inv_sqrt_det =
            Y0.invSqrtDet[idx] /
            sqrtf(mean_min_dists * mean_min_dists * mean_min_dists);
        Y_curr.invSqrtDet[idx] = clampFinitePositive(inv_sqrt_det, kMaxInvSqrtDet);
    }

    /// @brief Step 5: Launch Kernels for updating the covariances for both X0 and Y0
    /// @param device_data Contains X0, Y0 raw point clouds and X_curr, Y_curr buffers
    /// @param mean_min_dists Scalar scalinf factor to be used
    void launchUpdateCovariancesKernel(
        DeviceRegistrationData& device_data,
        float mean_min_dists
    )
    {
        /*
        // to be removed later: begin
        constexpr float kMinMeanMinDists = 1.0e-6f;
        if(!std::isfinite(mean_min_dists) || mean_min_dists <= kMinMeanMinDists)
        {
            std::fprintf(
                stderr,
                "[bot_dugma::e_step] WARNING: suspicious mean_min_dists=%.9e, clamping to %.9e\n",
                static_cast<double>(mean_min_dists),
                static_cast<double>(kMinMeanMinDists)
            );
            mean_min_dists = kMinMeanMinDists;
        }
        // to be removed later: end

        */
        constexpr int t_per_blk = 256;
        const int blk_count_X = static_cast<int>(
            ((device_data.XCurr().count + t_per_blk -1)/t_per_blk)
        );
        const int blk_count_Y = static_cast<int>(
            ((device_data.YCurr().count + t_per_blk -1)/t_per_blk)
        );

        updateXCovariancesKernel<<<blk_count_X, t_per_blk>>>(
            device_data.X0(),
            device_data.XCurr()
        );

        checkCuda(cudaGetLastError(), "updateXCovariancesKernel launch");
        checkCuda(cudaDeviceSynchronize(), "updateXCovariancesKernel sync");

        updateYCovariancesKernel<<<blk_count_Y, t_per_blk>>>(
            device_data.Y0(),
            device_data.YCurr(),
            mean_min_dists
        );

        checkCuda(cudaGetLastError(), "updateYCovariancesKernel launch");
        checkCuda(cudaDeviceSynchronize(), "updateYCovariancesKernel sync");

        printUpdatedCovarianceSummaries(device_data);
    }
    /// End of Step 5 ----------------------------------------------------------

    /// Step 6: Compute C_ij^old -----------------------------------------------

    /// @brief Compute paiwise Coefficient C_ij^old as per Eq. 18 in paper
    /// @param X_curr Buffer Point Cloud corresponding to X0
    /// @param Y_curr Buffer Point Cloud corresponding to Y0
    /// @param C_old Flattened float array to store N*M C_ij^old coefficients
    /// @return 
    __global__ void computeCOldCoeffsKernel(
        DeviceBufferPointCloud X_curr,
        DeviceBufferPointCloud Y_curr,
        float* C_old
    )
    {
        int idx = blockDim.x*blockIdx.x + threadIdx.x;
        int idy = blockDim.y*blockIdx.y + threadIdx.y;

        if(idx < X_curr.count && idy < Y_curr.count)
        {
            // call the pair (x_n, y_m) in the register
            const float Xx = X_curr.x[idx];
            const float Xy = X_curr.y[idx];
            const float Xz = X_curr.z[idx];

            const float Xisxx = X_curr.iSxx[idx];
            const float Xisxy = X_curr.iSxy[idx];
            const float Xisxz = X_curr.iSxz[idx];
            const float Xisyy = X_curr.iSyy[idx];
            const float Xisyz = X_curr.iSyz[idx];
            const float Xiszz = X_curr.iSzz[idx];

            const float XinvSqrtDet = X_curr.invSqrtDet[idx];

            const float Yx = Y_curr.x[idy];
            const float Yy = Y_curr.y[idy];
            const float Yz = Y_curr.z[idy];

            const float Yisxx = Y_curr.iSxx[idy];
            const float Yisxy = Y_curr.iSxy[idy];
            const float Yisxz = Y_curr.iSxz[idy];
            const float Yisyy = Y_curr.iSyy[idy];
            const float Yisyz = Y_curr.iSyz[idy];
            const float Yiszz = Y_curr.iSzz[idy];

            const float YinvSqrtDet = Y_curr.invSqrtDet[idy];

            // now the computations
            float dx = (Yx - Xx);
            float dy = (Yy - Xy);
            float dz = (Yz - Xz);
            float a = dx*dx;
            float b = 2*dx*dy;
            float c = 2*dx*dz;
            float d = dy*dy;
            float e = 2*dy*dz;
            float f = dz*dz;
            float Xpow = -0.5f*(a*Xisxx + b*Xisxy + c*Xisxz + d*Xisyy + e*Xisyz + f*Xiszz);
            float Ypow = -0.5f*(a*Yisxx + b*Yisxy + c*Yisxz + d*Yisyy + e*Yisyz + f*Yiszz);

            C_old[idy * X_curr.count + idx] =
                pi_factor * XinvSqrtDet * YinvSqrtDet * (expf(Xpow) + expf(Ypow));
        }
    }

    /// @brief Launches kernel for C_ij^old compute
    /// @param device_data Contains X0, Y0 raw point clouds and X_curr, Y_curr buffers
    void launchComputeCOldCoeffsKernel(
        DeviceRegistrationData& device_data
    )
    {
        constexpr int bx {32}, by {32};
        dim3 dim_block(bx, by, 1);
        // launch grid over an MxN array
        dim3 dim_grid(
            static_cast<int>((device_data.XCurr().count + bx - 1)/bx),
            static_cast<int>((device_data.YCurr().count + by - 1)/by),
            1
        );

        computeCOldCoeffsKernel<<<dim_grid, dim_block>>>(
            device_data.XCurr(),
            device_data.YCurr(),
            device_data.COld()
        );

        checkCuda(cudaGetLastError(), "computeCOldCoeffsKernel launch");
        checkCuda(cudaDeviceSynchronize(), "computeCOldCoeffsKernel sync");

        printArraySummary(
            "C_old",
            device_data.COld(),
            device_data.COldRows() * device_data.COldCols()
        );
    }
    /// End of Step 6 ----------------------------------------------------------


    /// Step 7: Coefficients for Energy Formulataion for M-Step ----------------
    /*
     0) R, t are to be estimated from optimization
        So let u = (tx,ty,tz,r0,r1,r2,r3,r4,r5,r6,r7,r8) = {u_p} for p=1, ..,12

     1) L = sum_i sum_j [ L_ij ] , for i=1,..., N and j = 1, .., M

     2) L_ij = C_ij*(y'_j - x_i).T*(Sig_i^-1 + Sig'_j^-1)*(y'_j - x_i)
        where,
        y'_j = R*y_j + t 
        Sig'_j^-1 = R*Sig_j^-1*R.T

     3) Separating (2) between point-pair quantities and u, such that
        L_ij(u) = C_ij * sum_k [ G_ijk*B_k(u) ]

     4) Substituting (3) into (1) to get,
        L =  sum_i sum_j [L_ij] = sum_i sum_j { C_ij * sum_k [ G_ijk*B_k(u) ] }
        Rearainging the summations to get,
        L = sum_k { sum_i sum_j [ C_ij*G_ijk ]*B_k(u) }

     5) Let some A_k = sum_i sum_j [ C_ij*G_ijk ]
        Then (4) can be re-written as,
        L = sum_k [ A_k*B_k(u) ] = A.T * B(u)

     Here k is the index of a term in the u-vector polynomial (see m_step_opt)
     Basically,
     L = a0*b0 + ... + ak*bk + ... aK*bK , for K = 637
     where ak = some expression in x_n, y_m and their respective covarinaces
     and bk = Pi_p[u_p^q] , for p = 1,..., 12 and q = 0,..., 4
     */

    constexpr int K {BasisCoeffsVec::kSize};
    #include "generated/G_K_tile_7.inl"
    #include "generated/B.inl"

    namespace
    {
        constexpr int kMaxEnergyConsistencyDiagnostics {3};
        constexpr std::size_t kMaxEnergyConsistencyPairs {250000};
        int g_energy_consistency_diagnostics_printed {0};

        struct HostBufferPointCloud
        {
            std::vector<float> x {};
            std::vector<float> y {};
            std::vector<float> z {};
            std::vector<float> iSxx {};
            std::vector<float> iSxy {};
            std::vector<float> iSxz {};
            std::vector<float> iSyy {};
            std::vector<float> iSyz {};
            std::vector<float> iSzz {};
        };

        void copyBufferPointCloudToHost(
            HostBufferPointCloud& dst,
            const DeviceBufferPointCloud& src,
            const char* label
        )
        {
            copyToHost(dst.x, src.x, src.count, label);
            copyToHost(dst.y, src.y, src.count, label);
            copyToHost(dst.z, src.z, src.count, label);
            copyToHost(dst.iSxx, src.iSxx, src.count, label);
            copyToHost(dst.iSxy, src.iSxy, src.count, label);
            copyToHost(dst.iSxz, src.iSxz, src.count, label);
            copyToHost(dst.iSyy, src.iSyy, src.count, label);
            copyToHost(dst.iSyz, src.iSyz, src.count, label);
            copyToHost(dst.iSzz, src.iSzz, src.count, label);
        }

        double polynomialEnergy(
            const BasisCoeffsVec& A,
            const std::array<float, 12>& u
        )
        {
            std::array<float, BasisCoeffsVec::kSize> B {};
            computeBasisB(u, B);

            double energy {0.0};
            for(std::size_t k = 0; k < A.vals.size(); ++k)
            {
                energy +=
                    static_cast<double>(A.vals[k]) *
                    static_cast<double>(B[k]);
            }

            return energy;
        }

        double directEnergy(
            const HostBufferPointCloud& X,
            const HostBufferPointCloud& Y,
            const std::vector<float>& C_old,
            std::size_t N,
            std::size_t M,
            const std::array<float, 12>& u,
            float normalizer
        )
        {
            const double tx = u[0];
            const double ty = u[1];
            const double tz = u[2];

            const double R[9] {
                u[3], u[4], u[5],
                u[6], u[7], u[8],
                u[9], u[10], u[11]
            };

            double energy {0.0};

            for(std::size_t j = 0; j < M; ++j)
            {
                const double SY[9] {
                    Y.iSxx[j], Y.iSxy[j], Y.iSxz[j],
                    Y.iSxy[j], Y.iSyy[j], Y.iSyz[j],
                    Y.iSxz[j], Y.iSyz[j], Y.iSzz[j]
                };

                double RSY[9] {};
                double RSRt[9] {};
                for(int r = 0; r < 3; ++r)
                {
                    for(int c = 0; c < 3; ++c)
                    {
                        for(int k = 0; k < 3; ++k)
                        {
                            RSY[3 * r + c] += R[3 * r + k] * SY[3 * k + c];
                        }
                    }
                }
                for(int r = 0; r < 3; ++r)
                {
                    for(int c = 0; c < 3; ++c)
                    {
                        for(int k = 0; k < 3; ++k)
                        {
                            RSRt[3 * r + c] += RSY[3 * r + k] * R[3 * c + k];
                        }
                    }
                }

                const double Ytx =
                    R[0] * Y.x[j] + R[1] * Y.y[j] + R[2] * Y.z[j] + tx;
                const double Yty =
                    R[3] * Y.x[j] + R[4] * Y.y[j] + R[5] * Y.z[j] + ty;
                const double Ytz =
                    R[6] * Y.x[j] + R[7] * Y.y[j] + R[8] * Y.z[j] + tz;

                for(std::size_t i = 0; i < N; ++i)
                {
                    const double dx = Ytx - X.x[i];
                    const double dy = Yty - X.y[i];
                    const double dz = Ytz - X.z[i];

                    const double Sxx = X.iSxx[i] + RSRt[0];
                    const double Sxy = X.iSxy[i] + RSRt[1];
                    const double Sxz = X.iSxz[i] + RSRt[2];
                    const double Syy = X.iSyy[i] + RSRt[4];
                    const double Syz = X.iSyz[i] + RSRt[5];
                    const double Szz = X.iSzz[i] + RSRt[8];

                    const double quad =
                        dx * (Sxx * dx + Sxy * dy + Sxz * dz) +
                        dy * (Sxy * dx + Syy * dy + Syz * dz) +
                        dz * (Sxz * dx + Syz * dy + Szz * dz);

                    energy +=
                        static_cast<double>(C_old[j * N + i]) * quad;
                }
            }

            return energy * static_cast<double>(normalizer);
        }

        void printEnergyConsistency(
            const char* label,
            const BasisCoeffsVec& A,
            const HostBufferPointCloud& X,
            const HostBufferPointCloud& Y,
            const std::vector<float>& C_old,
            std::size_t N,
            std::size_t M,
            const std::array<float, 12>& u,
            float normalizer
        )
        {
            const double direct =
                directEnergy(X, Y, C_old, N, M, u, normalizer);
            const double poly = polynomialEnergy(A, u);
            const double abs_diff = std::fabs(direct - poly);
            const double denom =
                std::fmax(1.0, std::fmax(std::fabs(direct), std::fabs(poly)));
            const double rel_diff = abs_diff / denom;

            std::fprintf(
                stderr,
                "[bot_dugma::e_step] energy consistency label=%s "
                "direct=%.9e poly=%.9e abs_diff=%.9e rel_diff=%.9e pairs=%zu\n",
                label,
                direct,
                poly,
                abs_diff,
                rel_diff,
                N * M
            );
        }

        void printEnergyConsistencyDiagnostics(
            const DeviceRegistrationData& device_data,
            const BasisCoeffsVec& A,
            float normalizer
        )
        {
            if(g_energy_consistency_diagnostics_printed >=
               kMaxEnergyConsistencyDiagnostics)
            {
                return;
            }

            const std::size_t N = device_data.XCurr().count;
            const std::size_t M = device_data.YCurr().count;
            const std::size_t pair_count = N * M;

            if(pair_count == 0 || pair_count > kMaxEnergyConsistencyPairs)
            {
                std::fprintf(
                    stderr,
                    "[bot_dugma::e_step] energy consistency skipped pairs=%zu "
                    "limit=%zu\n",
                    pair_count,
                    kMaxEnergyConsistencyPairs
                );
                ++g_energy_consistency_diagnostics_printed;
                return;
            }

            ++g_energy_consistency_diagnostics_printed;

            HostBufferPointCloud X {};
            HostBufferPointCloud Y {};
            std::vector<float> C_old {};

            copyBufferPointCloudToHost(
                X,
                device_data.XCurr(),
                "copy X_curr energy consistency"
            );
            copyBufferPointCloudToHost(
                Y,
                device_data.YCurr(),
                "copy Y_curr energy consistency"
            );
            copyToHost(
                C_old,
                device_data.COld(),
                pair_count,
                "copy C_old energy consistency"
            );

            const std::array<float, 12> identity {
                0.0f, 0.0f, 0.0f,
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f
            };

            const std::array<float, 12> small_translation {
                0.01f, -0.005f, 0.0f,
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f
            };

            const float yaw = 0.01f;
            const float c = cosf(yaw);
            const float s = sinf(yaw);
            const std::array<float, 12> small_yaw {
                0.0f, 0.0f, 0.0f,
                c, -s, 0.0f,
                s, c, 0.0f,
                0.0f, 0.0f, 1.0f
            };

            printEnergyConsistency(
                "identity", A, X, Y, C_old, N, M, identity, normalizer
            );
            printEnergyConsistency(
                "small_translation",
                A,
                X,
                Y,
                C_old,
                N,
                M,
                small_translation,
                normalizer
            );
            printEnergyConsistency(
                "small_yaw", A, X, Y, C_old, N, M, small_yaw, normalizer
            );
        }

        void normalizeBasisCoeffs(BasisCoeffsVec& A)
        {
            float max_abs {0.0f};

            for(const float value : A.vals)
            {
                if(std::isfinite(value))
                {
                    max_abs = fmaxf(max_abs, fabsf(value));
                }
            }

            if(!(std::isfinite(max_abs) && max_abs > 0.0f))
            {
                std::fprintf(
                    stderr,
                    "[bot_dugma::e_step] A normalization skipped max_abs=%.9e\n",
                    static_cast<double>(max_abs)
                );
                return;
            }

            const float inv_max_abs = 1.0f / max_abs;
            for(float& value : A.vals)
            {
                value *= inv_max_abs;
            }

            std::fprintf(
                stderr,
                "[bot_dugma::e_step] A normalized max_abs_before=%.9e "
                "scale=%.9e max_abs_after=1.000000000e+00\n",
                static_cast<double>(max_abs),
                static_cast<double>(inv_max_abs)
            );
        }
    }

    template<int K_tile, int pairs_per_t, int t_per_blk>
    __global__ void computeAPartialCoeffsKernel(
        DeviceBufferPointCloud X_curr,
        DeviceBufferPointCloud Y_curr,
        const float* __restrict__ C_old,
        const int num_chunks,
        float* __restrict__  A_partial
    )
    {
        const int N = static_cast<int>(X_curr.count);
        const int M = static_cast<int>(Y_curr.count);

        // fisrt k of this block 
        const int k0 = blockIdx.x * K_tile; 
         
        const int tid = threadIdx.x;
        // P = N*M
        const int P = N * M;
        // Pairs P divided into chunks of size pairs_per_t * t_per_blk
        const int pairs_per_blk = t_per_blk * pairs_per_t;
        // index of that chunk, such that c = 0,...,num_chunks
        const int c = blockIdx.y;
        // first pair in chunk c, local id
        const int p0 = c * pairs_per_blk;

        // Each thread owns an array for each k_tile
        float A_thread[K_tile];
        #pragma unroll
        // make it zero before start of each block
        for(int c_k {0}; c_k < K_tile; ++c_k)
        {
            A_thread[c_k] = 0.0f;
        }

        // Each thread handles pair_per_t pairs
        #pragma unroll
        for(int c_p {0}; c_p < pairs_per_t; ++c_p)
        {
            // p is the global id, so p = 0,...,P over a flattened array
            // So to iterate over all (x_i, y_j), 
            // p := j * N + i
            const int p = p0 + tid + t_per_blk * c_p;

            if(p < P)
            {
                const int i = p % N;
                const int j = p / N;

                // load the point pair dependent quantities into the register
                const float Xx = X_curr.x[i];
                const float Xy = X_curr.y[i];
                const float Xz = X_curr.z[i];

                const float Xisxx = X_curr.iSxx[i];
                const float Xisxy = X_curr.iSxy[i];
                const float Xisxz = X_curr.iSxz[i];
                const float Xisyy = X_curr.iSyy[i];
                const float Xisyz = X_curr.iSyz[i];
                const float Xiszz = X_curr.iSzz[i];

                const float Yx = Y_curr.x[j];
                const float Yy = Y_curr.y[j];
                const float Yz = Y_curr.z[j];

                const float Yisxx = Y_curr.iSxx[j];
                const float Yisxy = Y_curr.iSxy[j];
                const float Yisxz = Y_curr.iSxz[j];
                const float Yisyy = Y_curr.iSyy[j];
                const float Yisyz = Y_curr.iSyz[j];
                const float Yiszz = Y_curr.iSzz[j];

                const float C = C_old[j * N + i];

                // for this K_tile indexed by k0
                // compute G_tile[k0] = ( G[k0],..., G[k0 + (K_tile -1)] )
                // then compute the A_thread array for this thread
                accumulateGTileK7(
                    blockIdx.x,
                    Xx, Xy, Xz,
                    Yx, Yy, Yz,
                    Xisxx, Xisxy, Xisxz,
                    Xisyy, Xisyz, Xiszz,
                    Yisxx, Yisxy, Yisxz,
                    Yisyy, Yisyz, Yiszz,
                    C,
                    A_thread
                );
            }
        }

        // Reduce A_thread across all threads in this block, to 
        __shared__ float A_blk[K_tile][t_per_blk];

        // for this block
        #pragma unroll
        for(int c_k {0}; c_k < K_tile; ++c_k)
        {
            A_blk[c_k][tid] = A_thread[c_k];
        }
        __syncthreads();

        for(int step {t_per_blk/2}; step > 0; step = step/2)
        {
            if(tid < step)
            {
                #pragma unroll
                for(int c_k {0}; c_k < K_tile; ++c_k)
                {
                    A_blk[c_k][tid] += A_blk[c_k][tid + step];
                }
            }
            __syncthreads();
        }

        // One thread writes this bloack's partial sums
        if(tid == 0)
        {
            #pragma unroll
            for(int c_k {0}; c_k < K_tile; ++c_k)
            {
                const int k = k0 + c_k;
                if(k < K)
                {
                    A_partial[k * num_chunks + c] = A_blk[c_k][0];
                }
            }
        }
    }

    template<int t_per_blk>
    __global__ void reduceAPartialsKernel(
        const float* __restrict__ A_partial,
        const int num_chunks,
        const float normalizer,
        float* __restrict__ A
    )
    {
        const int k = blockIdx.x;
        const int tid = threadIdx.x;

        __shared__ float sdata[t_per_blk];

        float sum = 0.0f;

        for(int c {tid}; c < num_chunks; c += t_per_blk)
        {
            sum += A_partial[k * num_chunks + c];
        }

        sdata[tid] = sum;
        __syncthreads();

        for (int step {t_per_blk/2}; step > 0; step = step/2)
        {
            if(tid < step)
            {
                sdata[tid] += sdata[tid + step];
            }
            __syncthreads();
        }

        if(tid == 0)
        {
            A[k] = sdata[0] * normalizer;
        }
    }

    BasisCoeffsVec launchComputeACoeffsKernel(
        DeviceRegistrationData& device_data
    )
    {
        constexpr int K_tile = 7;
        constexpr int pairs_per_t = 8;
        constexpr int t_per_blk = 256;
        constexpr int pairs_per_blk = pairs_per_t * t_per_blk;

        const int num_chunks = static_cast<int>(
            (device_data.XCurr().count*device_data.YCurr().count + pairs_per_blk -1)
            / pairs_per_blk
        );
        const float pair_count =
            static_cast<float>(device_data.XCurr().count) *
            static_cast<float>(device_data.YCurr().count);
        const float normalizer = pair_count > 0.0f ? 1.0f / pair_count : 0.0f;
        const int num_K_tiles = (K + K_tile -1)/K_tile;

        dim3 dim_grid1(num_K_tiles, num_chunks);
        dim3 dim_block1(t_per_blk);

        float* A_partial;

        allocateOnDevice(A_partial, K * num_chunks, "cudaMalloc A_partial");

        // Kernel 1
        computeAPartialCoeffsKernel<K_tile, pairs_per_t, t_per_blk><<<dim_grid1, dim_block1>>>(
            device_data.XCurr(),
            device_data.YCurr(),
            device_data.COld(),
            num_chunks,
            A_partial
        );

        checkCuda(cudaGetLastError(), "computeAPartialCoeffsKernel launch");
        checkCuda(cudaDeviceSynchronize(), "computeAPartialCoeffsKernel sync");

        
        dim3 dim_grid2(K);
        dim3 dim_block2(t_per_blk);

        // Kernel 2
        reduceAPartialsKernel<t_per_blk><<<dim_grid2, dim_block2>>>(
            A_partial,
            num_chunks,
            normalizer,
            device_data.A()
        );

        checkCuda(cudaGetLastError(), "reduceAPartialsKernel launch");
        checkCuda(cudaDeviceSynchronize(), "reduceAPartialsKernel sync");

        BasisCoeffsVec A_h {};

        copyToHost(
            A_h.vals.data(),
            device_data.A(),
            K,
            "copy A to host"
        );

        printEnergyConsistencyDiagnostics(
            device_data,
            A_h,
            normalizer
        );

        normalizeBasisCoeffs(A_h);

        freeOnDevice(A_partial);

        return A_h;
    }

    
}
