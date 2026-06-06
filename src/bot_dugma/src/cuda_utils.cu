#include "cuda_utils.hpp"

#include <stdexcept>
#include <string>

namespace bot_dugma
{
    void checkCuda(
        cudaError_t error,
        const char* context
    )
    {
        if (error != cudaSuccess) {
            throw std::runtime_error(
                std::string(context) + ": " + cudaGetErrorString(error)
            );
        }
    }

    void allocateOnDevice(
        float*& ptr,
        std::size_t count,
        const char* name
    )
    {
        checkCuda(
            cudaMalloc(reinterpret_cast<void**>(&ptr), count * sizeof(float)),
            name
        );
    }

    void copyToDevice(
        float* dst,
        const std::vector<float>& src,
        const char* name
    )
    {
        checkCuda(
            cudaMemcpy(
                dst,
                src.data(),
                src.size() * sizeof(float),
                cudaMemcpyHostToDevice
            ),
            name
        );
    }

    void copyToHost(
        std::vector<float>& dst,
        const float* src,
        std::size_t count,
        const char* name
    )
    {
        dst.resize(count);
        checkCuda(
            cudaMemcpy(
                dst.data(),
                src,
                count * sizeof(float),
                cudaMemcpyDeviceToHost
            ),
            name
        );
    }

    void copyToHost(
        float* dst,
        const float* src,
        std::size_t count,
        const char* name
    )
    {
        checkCuda(
            cudaMemcpy(
                dst,
                src,
                count * sizeof(float),
                cudaMemcpyDeviceToHost
            ),
            name
        );
    }

    void freeOnDevice(float*& ptr)
    {
        if (ptr == nullptr) {
            return;
        }

        checkCuda(
            cudaFree(ptr),
            "cudaFree"
        );

        ptr = nullptr;
    }
}
