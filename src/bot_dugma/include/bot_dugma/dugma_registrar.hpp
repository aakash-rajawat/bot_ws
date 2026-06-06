#ifndef DUGMA_REGISTRAR_HPP
#define DUGMA_REGISTRAR_HPP

#include "bot_dugma/types.hpp"

namespace bot_dugma
{
    class DugmaRegistrar
    {
    public:
        explicit DugmaRegistrar(const DugmaRegistrarConfig& config);

        PoseEstimate estimate(
            const PointCloud& in_X,
            const PointCloud& in_Y,
            const PoseEstimate* init_guess = nullptr
        );

        PoseEstimate estimate(
            const PointCloudView& in_X,
            const PointCloudView& in_Y,
            const PoseEstimate* init_guess = nullptr
        );


    private:
        DugmaRegistrarConfig m_config {};
    };
    
    
}

#endif
