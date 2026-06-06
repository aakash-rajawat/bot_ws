#include "bot_dugma/initialize.hpp"
#include "initialize_cuda.hpp"

namespace bot_dugma
{
    DeviceRegistrationData::DeviceRegistrationData(
        const PointCloud& in_X,
        const PointCloud& in_Y
    )
    {
        m_X0.allocateAndCopyFromHost(in_X);
        m_Y0.allocateAndCopyFromHost(in_Y);
        m_X_curr.allocateBufferOnDevice(m_X0);
        m_Y_curr.allocateBufferOnDevice(m_Y0);

        m_C_old_rows = m_Y_curr.count;
        m_C_old_cols = m_X_curr.count;
        allocateOnDevice(m_C_old, m_C_old_rows*m_C_old_cols, "cudaMalloc m_C_old");

        constexpr int kSize = 637;
        allocateOnDevice(m_A, kSize, "cudaMalloc m_A");

        precomputeCovarianceTerms(m_X0);
        precomputeCovarianceTerms(m_Y0);
    }

    void DevicePointCloud::allocateAndCopyFromHost(const PointCloud& in)
    {
        count = in.m_x.size();
        if (count == 0) {
            return;
        }

        allocateOnDevice(x, count, "cudaMalloc x");
        allocateOnDevice(y, count, "cudaMalloc y");
        allocateOnDevice(z, count, "cudaMalloc z");

        allocateOnDevice(Sxx, count, "cudaMalloc Sxx");
        allocateOnDevice(Sxy, count, "cudaMalloc Sxy");
        allocateOnDevice(Sxz, count, "cudaMalloc Sxz");
        allocateOnDevice(Syy, count, "cudaMalloc Syy");
        allocateOnDevice(Syz, count, "cudaMalloc Syz");
        allocateOnDevice(Szz, count, "cudaMalloc Szz");

        allocateOnDevice(iSxx, count, "cudaMalloc iSxx");
        allocateOnDevice(iSxy, count, "cudaMalloc iSxy");
        allocateOnDevice(iSxz, count, "cudaMalloc iSxz");
        allocateOnDevice(iSyy, count, "cudaMalloc iSyy");
        allocateOnDevice(iSyz, count, "cudaMalloc iSyz");
        allocateOnDevice(iSzz, count, "cudaMalloc iSzz");

        allocateOnDevice(invSqrtDet, count, "cudaMalloc invSqrtDet");

        copyToDevice(x, in.m_x, "cudaMemcpy x");
        copyToDevice(y, in.m_y, "cudaMemcpy y");
        copyToDevice(z, in.m_z, "cudaMemcpy z");

        copyToDevice(Sxx, in.m_Sxx, "cudaMemcpy Sxx");
        copyToDevice(Sxy, in.m_Sxy, "cudaMemcpy Sxy");
        copyToDevice(Sxz, in.m_Sxz, "cudaMemcpy Sxz");
        copyToDevice(Syy, in.m_Syy, "cudaMemcpy Syy");
        copyToDevice(Syz, in.m_Syz, "cudaMemcpy Syz");
        copyToDevice(Szz, in.m_Szz, "cudaMemcpy Szz");
    }

    void DeviceBufferPointCloud::allocateBufferOnDevice(const DevicePointCloud& reference)
    {
        count = reference.count;
        if (count == 0) {
            return;
        }

        allocateOnDevice(x, count, "cudaMalloc x");
        allocateOnDevice(y, count, "cudaMalloc y");
        allocateOnDevice(z, count, "cudaMalloc z");

        allocateOnDevice(iSxx, count, "cudaMalloc iSxx");
        allocateOnDevice(iSxy, count, "cudaMalloc iSxy");
        allocateOnDevice(iSxz, count, "cudaMalloc iSxz");
        allocateOnDevice(iSyy, count, "cudaMalloc iSyy");
        allocateOnDevice(iSyz, count, "cudaMalloc iSyz");
        allocateOnDevice(iSzz, count, "cudaMalloc iSzz");

        allocateOnDevice(invSqrtDet, count, "cudaMalloc invSqrtDet");
    }

    void DeviceRegistrationData::precomputeCovarianceTerms(
        DevicePointCloud& pt_cloud
    )
    {
        // decide block and grid dimensions
        constexpr int t_per_blk = 256;
        const int blk_count = static_cast<int>(
            (pt_cloud.count + t_per_blk -1)/t_per_blk
        );

        // call main kernel function
        launchPrecomputeCovarianceTermsKernel(
            pt_cloud,
            blk_count,
            t_per_blk
        );
    }

    void DevicePointCloud::release()
    {
        freeOnDevice(x);
        freeOnDevice(y);
        freeOnDevice(z);

        freeOnDevice(Sxx);
        freeOnDevice(Sxy);
        freeOnDevice(Sxz);
        freeOnDevice(Syy);
        freeOnDevice(Syz);
        freeOnDevice(Szz);

        freeOnDevice(iSxx);
        freeOnDevice(iSxy);
        freeOnDevice(iSxz);
        freeOnDevice(iSyy);
        freeOnDevice(iSyz);
        freeOnDevice(iSzz);

        freeOnDevice(invSqrtDet);

        count = 0;
    }

    void DeviceBufferPointCloud::release()
    {
        freeOnDevice(x);
        freeOnDevice(y);
        freeOnDevice(z);

        freeOnDevice(iSxx);
        freeOnDevice(iSxy);
        freeOnDevice(iSxz);
        freeOnDevice(iSyy);
        freeOnDevice(iSyz);
        freeOnDevice(iSzz);

        freeOnDevice(invSqrtDet);

        count = 0;
    }

    DeviceRegistrationData::~DeviceRegistrationData()
    {
        m_X0.release();
        m_Y0.release();
        m_X_curr.release();
        m_Y_curr.release();

        freeOnDevice(m_C_old);
        m_C_old_rows = 0;
        m_C_old_cols = 0;

        freeOnDevice(m_A);
    }

    const DevicePointCloud& DeviceRegistrationData::X0() const noexcept
    {
        return m_X0;
    }

    const DevicePointCloud& DeviceRegistrationData::Y0() const noexcept
    {
        return m_Y0;
    }

    DeviceBufferPointCloud& DeviceRegistrationData::XCurr() noexcept
    {
        return m_X_curr;
    }

    const DeviceBufferPointCloud& DeviceRegistrationData::XCurr() const noexcept
    {
        return m_X_curr;
    }

    DeviceBufferPointCloud& DeviceRegistrationData::YCurr() noexcept
    {
        return m_Y_curr;
    }

    const DeviceBufferPointCloud& DeviceRegistrationData::YCurr() const noexcept
    {
        return m_Y_curr;
    }

    float* DeviceRegistrationData::COld() noexcept
    {
        return m_C_old;
    }

    const float* DeviceRegistrationData::COld() const noexcept
    {
        return m_C_old;
    }

    std::size_t DeviceRegistrationData::COldRows() const noexcept
    {
        return m_C_old_rows;
    }

    std::size_t DeviceRegistrationData::COldCols() const noexcept
    {
        return m_C_old_cols;
    }

    float* DeviceRegistrationData::A() noexcept
    {
        return m_A;
    }

    const float* DeviceRegistrationData::A() const noexcept
    {
        return m_A;
    }
}
