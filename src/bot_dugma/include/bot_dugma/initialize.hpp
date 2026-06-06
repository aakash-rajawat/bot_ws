#ifndef INITIALIZE_HPP
#define INITIALIZE_HPP

#include "bot_dugma/types.hpp"

#include <array>

namespace bot_dugma
{
    struct DevicePointCloud
    {
        float* x = nullptr;
        float* y = nullptr;
        float* z = nullptr;

        float* Sxx = nullptr;
        float* Sxy = nullptr;
        float* Sxz = nullptr;
        float* Syy = nullptr;
        float* Syz = nullptr;
        float* Szz = nullptr;

        float* iSxx = nullptr;
        float* iSxy = nullptr;
        float* iSxz = nullptr;
        float* iSyy = nullptr;
        float* iSyz = nullptr;
        float* iSzz = nullptr;

        float* invSqrtDet = nullptr;

        std::size_t count = 0;

        void allocateAndCopyFromHost(const PointCloud& in);
        void release();
    };

    struct DeviceBufferPointCloud
    {
        float* x = nullptr;
        float* y = nullptr;
        float* z = nullptr;

        float* iSxx = nullptr;
        float* iSxy = nullptr;
        float* iSxz = nullptr;
        float* iSyy = nullptr;
        float* iSyz = nullptr;
        float* iSzz = nullptr;

        float* invSqrtDet = nullptr;

        std::size_t count = 0;

        void allocateBufferOnDevice(const DevicePointCloud& reference);
        void release();
    };

    class DeviceRegistrationData
    {
    public:
        explicit DeviceRegistrationData(
            const PointCloud& in_X,
            const PointCloud& in_Y
        );
        ~DeviceRegistrationData();

        DeviceRegistrationData(const DeviceRegistrationData&) = delete;
        DeviceRegistrationData& operator=(const DeviceRegistrationData&) = delete;

        const DevicePointCloud& X0() const noexcept;
        const DevicePointCloud& Y0() const noexcept;
        DeviceBufferPointCloud& XCurr() noexcept;
        const DeviceBufferPointCloud& XCurr() const noexcept;
        DeviceBufferPointCloud& YCurr() noexcept;
        const DeviceBufferPointCloud& YCurr() const noexcept;

        float* COld() noexcept;
        const float* COld() const noexcept;
        std::size_t COldRows() const noexcept;
        std::size_t COldCols() const noexcept;

        float* A() noexcept;
        const float* A() const noexcept;


    private:
        void precomputeCovarianceTerms(DevicePointCloud& pt_cloud);

        DevicePointCloud m_X0 {};
        DevicePointCloud m_Y0 {};
        DeviceBufferPointCloud m_X_curr {};
        DeviceBufferPointCloud m_Y_curr {};

        float* m_C_old = nullptr;
        std::size_t m_C_old_rows = 0;
        std::size_t m_C_old_cols = 0;

        float* m_A = nullptr;
    };
}

#endif
