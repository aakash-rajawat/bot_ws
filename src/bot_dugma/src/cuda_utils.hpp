#ifndef CUDA_UTILS_HPP
#define CUDA_UTILS_HPP

#include <cuda_runtime.h>

#include <cstddef>
#include <vector>

namespace bot_dugma
{
    void checkCuda(
        cudaError_t error,
        const char* context
    );

    void allocateOnDevice(
        float*& ptr,
        std::size_t count,
        const char* name
    );

    void copyToDevice(
        float* dst,
        const std::vector<float>& src,
        const char* name
    );

    void copyToHost(
        std::vector<float>& dst,
        const float* src,
        std::size_t count,
        const char* name
    );

    void copyToHost(
        float* dst,
        const float* src,
        std::size_t count,
        const char* name
    );

    void freeOnDevice(float*& ptr);
}

#endif
